#include <actionlib/client/simple_action_client.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <cmath>
#include <cstdlib>          // Needed for rand()
#include <ctime>            // Needed to seed random number generator with a time value
#include <fstream>
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include "move_base_msgs/MoveBaseAction.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <nav_msgs/GetPlan.h>
#include "std_msgs/Int32.h"
#include <ostream>
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include "controller/fd/ffd.h"
#include "controller/fd/wfd.h"
#include "visualization_msgs/Marker.h"

class Controller {

public:
    // Construst a new RandomWalk object and hook up this ROS node
    // to the simulated robot's velocity control and laser topics
    Controller(ros::NodeHandle& nh) :
        fsm(FSM_RND_MOVE_FORWARD),rotateStartTime(ros::Time::now()),rotateDuration(0.f) {
        // Initialize random time generator
        srand(time(NULL));

        // Advertise a new publisher for the simulated robot's velocity command topic
        // (the second argument indicates that if multiple command messages are in
        //  the queue to be sent, only the last command will be sent)
        commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        pointPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        goalPointPub  = nh.advertise<visualization_msgs::Marker>("goal_marker", 10);
        // Subscribe to the simulated robot's laser scan topic and tell ROS to call
        // this->commandCallback() whenever a new message is published on that topic
        laserSub = nh.subscribe("base_scan", 1, &Controller::commandCallback, this);
        mapSub = nh.subscribe("map", 1, &Controller::mapCallback, this);
        odomSub = nh.subscribe("odom", 1, &Controller::odomCallback, this);
        goalSub = nh.subscribe("goalState", 1, &Controller::goalCallback, this);
        goalPub = nh.advertise<move_base_msgs::MoveBaseGoal>("goal", 1);
        hasNewMap = false;
    }

    /* Callback methods */

    // Callback from frontier navigation which reports whether a provided goal was reacheable/reached
    void goalCallback(const std_msgs::Int32 goalState){
        if(goalState.data == 0){
            ROS_ERROR("A goal was accepted");
        } else {
          ROS_ERROR("The goal stopped being interesting for some reason");
          destination.pos.x = 0;    // NULLING DESTINATION
          destination.pos.y = 0;
      }
    }

    // Callback for omodetry, to be integrated. (We should probably not do this -> noise?)
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        // X and Y translation coordinate from the origin, where the robot started
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double turn = tf::getYaw(msg->pose.pose.orientation);

        robot_pos.x = (mapSize[0] / 2) + ceil(x/mapResolution);
        robot_pos.y = (mapSize[1] / 2) + ceil(y/mapResolution);
        robot_pos.z = turn;
        //Print out current translated position of the robot
    }

    // Callback from gmapping with new map; triggers a update to frontier detection
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        this->map = msg;
        mapSize[0] = msg->info.width;
        mapSize[1] = msg->info.height;
        mapResolution = msg->info.resolution;
        hasNewMap = true;
    }

    // Process the incoming laser scan message; used in random navigation
    void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
        unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
        float closestRange = msg->ranges[minIndex];
        for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
            float currAngle = msg->angle_min + msg->angle_increment*currIndex;
            // choose closest range
            if (msg->ranges[currIndex] < closestRange && currAngle <= msg->angle_max) {
                closestRange = msg->ranges[currIndex];
            }
            // add element to sensorMeasurements
            _pose measurementPose;
            measurementPose.x = robot_pos.x;
            measurementPose.y = robot_pos.y;
            sensorMeasurements.push_back(measurementPose);
        }

        ROS_INFO_STREAM("Range: " << closestRange);
        ROS_INFO_STREAM("Angle: " << msg->angle_min);
        if(fsm == FSM_RND_MOVE_FORWARD && closestRange <= PROXIMITY_RANGE_M){
            fsm = FSM_RND_ROTATE;
            rotateStartTime = (ros::Time::now());
            rotateDuration = ros::Duration(rand()%4);
        }
    };

    // Checks whether the current target is still valid: (no path found in last iteration, too close to current robot position, Nulled by navigation callback or no longer a frontier)
    void checkTargetValidity(){
        bool cNull = (destination.pos.x==0 && destination.pos.y==0);
        if(targettingStack.empty() && cNull){
            ROS_ERROR("=== NO TARGETS AVAILABLE, NO DESTINATION SET ===");
            fsm = FSM_RND_MOVE_FORWARD;

            return;
        }

        fd::_pose pose = robot_pos;

        double dist = distance(destination.pos, pose);

        ROS_ERROR("Distance between destination and robot: %f", dist);

        bool cDist = dist < 3,
                cStillFrontier = std::find_if(lastFoundFrontiers.begin(), lastFoundFrontiers.end(), fd::same_pose(destination)) == lastFoundFrontiers.end();

        if( noPath
                || cDist   // check if in range of destination
                || cNull   // check if destination valid
                || cStillFrontier ){ // check if still a frontier

            ROS_ERROR("==============");
            ROS_ERROR("RESETTING PATH BECAUSE: %d, %d, %d, %d", noPath, cDist, cNull, cStillFrontier);
            ROS_ERROR("==============");

            destination.pos.x = 0;    // NULLING DESTINATION
            destination.pos.y = 0;

            noPath=false;

            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(1.0))){}

            ac.cancelAllGoals();

            if(targettingStack.empty()){
                ROS_ERROR("==============");
                ROS_ERROR("NO TARGETS AVAILABLE");
                ROS_ERROR("==============");
                fsm = FSM_RND_MOVE_FORWARD;
            } else {
                ROS_ERROR("==============");
                ROS_ERROR("RESETTING PATH BECAUSE: %d, %d, %d, %d", noPath, cDist, cNull, cStillFrontier);
                ROS_ERROR("==============");
                fsm = FSM_SELECT_NEW_GOAL;
            }
        }
    }

    // Runs the WFD algorithm on the last received map callback
    void updateFrontiers(){
        if(!hasNewMap){
            return;
        }

        hasNewMap = false; // TODO: THREAD - LOCK THESE TIMING ISSUES;

        fd::_pose pose = robot_pos;

        if(std::isnan(pose.x) || std::isnan(pose.y) || pose.x < 0 || pose.x >= map->info.width || pose.y < 0 || pose.y >= map->info.height) return;
        frontierDetector.updateMap(map);
        // TODO use real sensor measurements!!
        frontierDetector.update(wfd::nothing());
        std::vector<fd::_pose> frontiers = frontierDetector.frontierDetection(pose);
        lastFoundFrontiers = fd::sortFrontiers(fd::DIST_ROBOT, pose.x, pose.y, frontiers);
        targettingStack = lastFoundFrontiers;

        ROS_ERROR("Updating Map && Visualization");

        // Normalize the values in the sorted frontiers
        double minV = DBL_MAX, maxV = DBL_MIN;
        for(unsigned int i = 0 ; i < lastFoundFrontiers.size() ; ++i) {
            double val = lastFoundFrontiers[i].val;
            minV = val < minV ? val : minV;
            maxV = val > maxV ? val : maxV;
        }
        double range = maxV - minV;

        // visualization
        visualization_msgs::Marker points;
        points.header.stamp = ros::Time::now();
        points.header.frame_id = "/map";
        points.ns = "turtlebotSlam";
        points.pose.orientation.w = 1.0;
        points.action = visualization_msgs::Marker::ADD;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = mapResolution * 0.8f;
        points.scale.y = mapResolution * 0.8f;
        points.frame_locked=true;

        points.color.r = 0.01f;
        points.color.g = 0.01f;
        points.color.b = 0.01f;
        points.color.a = 1.0f;

        for(unsigned int i = 0 ; i < lastFoundFrontiers.size() ; ++i) {
            fd::ValuePose pose = lastFoundFrontiers[i];

            geometry_msgs::Point p;
            p.x = (pose.pos.x - mapSize[0]/2) * mapResolution - 7.5 * mapResolution;
            p.y = (pose.pos.y - mapSize[1]/2) * mapResolution - 7.5 * mapResolution;
            //p.x = (pose.pos.x - mapSize[0]/2) * mapResolution;
           // p.y = (pose.pos.y - mapSize[1]/2) * mapResolution;
            p.z = 0.0001;

            double norm = (pose.val - minV) / range;
            std_msgs::ColorRGBA color;

            color.r = 1 - norm;
            color.g = 0.0;
            color.b = norm - 1;
            color.a = 1.0;

            points.points.push_back(p);
            points.colors.push_back(color);
        }

        ROS_ERROR("Publishing Frontiers");
        pointPub.publish(points);
    }

    // Selects a new destination; the top 20% of the targetting stack is polled with random st. dist.; The selected goal is then removed from that stack
    void selectNewDestination(){
        if(!targettingStack.empty()){

            int sSize = targettingStack.size();
            sSize /= 5; // select top 20%

            double stdDev = sSize * 0.33;
            boost::normal_distribution<> oDistr(0,stdDev);
            boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varO(rng, oDistr);

            double v = varO();
            int k = abs(int(v));
            destination = targettingStack[k % sSize];
            targettingStack.erase(targettingStack.begin()+ (k % sSize));

            fsm = FSM_FOLLOW_GOAL;
            ROS_ERROR("Selected new destination: %f - %d :: %f-%f", v, k % sSize, destination.pos.x, destination.pos.y);

        } else {
            fsm = FSM_RND_MOVE_FORWARD;
            ROS_ERROR("targetting stack was empty!");
        }
    }

    // Tries to find a plan for the provided target; retrying five alternatives (calls to selectNewDestination).
    // If a valid plan is found; it publishes the target as a goal
    void findPlanFor(fd::_pose pose){

        ROS_ERROR("Trying the given destination");

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<nav_msgs::GetPlan>("/move_base_node/make_plan");
        nav_msgs::GetPlan srv;

        geometry_msgs::PoseStamped start_pose, goal_pose;

        tf::Quaternion q = tf::createQuaternionFromYaw(robot_pos.z);

        int tries = 5;
        bool found = false;
        do{
            ROS_ERROR("Trying to find path");

            --tries;
            start_pose.header.frame_id = "/map";
            start_pose.pose.position.x=(robot_pos.x - mapSize[0]/2) * mapResolution; //convergence from occupancygrid indexes to map coordinates
            start_pose.pose.position.y=(robot_pos.y - mapSize[1]/2) * mapResolution;
            start_pose.pose.orientation.w = q.getW();

            goal_pose.header.frame_id = "/map";
            goal_pose.pose.position.x=(pose.x - mapSize[0]/2) * mapResolution; //convergence from occupancygrid indexes to map coordinates
            goal_pose.pose.position.y=(pose.y - mapSize[1]/2) * mapResolution;
            goal_pose.pose.orientation.w = q.getW();

            srv.request.start = start_pose;
            srv.request.goal = goal_pose;

            found = false;

            if (client.call(srv)){
                if (srv.response.plan.poses.size() == 0) {
                    ROS_ERROR("... no plan found TRYING DIFFERENT!");

                    selectNewDestination();
                    pose = destination.pos;
                } else {
                    found = true;
                    ROS_ERROR("... found a plan!");
                }
            } else {
                ROS_ERROR("... no communication so returning (We're still moving)!");
                return;
            }
        } while(!found && tries >= 0);

        if(!found){            
            fsm = FSM_SELECT_NEW_GOAL;
            noPath = true;
            ROS_ERROR("... no goal found!");
            return;
        }

        MoveBaseClient ac("move_base", true);
        while(!ac.waitForServer(ros::Duration(1.0))){}

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x=(pose.x - mapSize[0]/2) * mapResolution  - 7.5 * mapResolution; //convergence from occupancygrid indexes to map coordinates
        goal.target_pose.pose.position.y=(pose.y - mapSize[1]/2) * mapResolution  - 7.5 * mapResolution;
        goal.target_pose.pose.orientation.w = 1.0;

        {
            visualization_msgs::Marker goalPoint;
            goalPoint.header.stamp = ros::Time::now();
            goalPoint.header.frame_id = "/map";
            goalPoint.ns = "turtlebotSlam";
            goalPoint.pose.orientation.w = 1.0;
            goalPoint.action = visualization_msgs::Marker::ADD;
            goalPoint.id = 0;
            goalPoint.type = visualization_msgs::Marker::POINTS;
            goalPoint.scale.x = mapResolution * 2.0f;
            goalPoint.scale.y = mapResolution * 2.0f;
            goalPoint.color.g = 1.0f;
            goalPoint.color.a = 1.0;

            geometry_msgs::Point p;
            p.x = (pose.x - mapSize[0]/2) * mapResolution - 7.5 * mapResolution;
            p.y = (pose.y - mapSize[1]/2) * mapResolution - 7.5 * mapResolution;
            //p.x = (destination.x - mapSize[0]/2) * mapResolution;
            //p.y= (destination.y - mapSize[1]/2) * mapResolution;
            p.z = 0.02;

            goalPoint.points.push_back(p);
            goalPoint.colors.push_back(goalPoint.color);
            goalPointPub.publish(goalPoint);

            ROS_ERROR("Published Goal Marker");
        }
        {
            visualization_msgs::Marker goalPoint;
            goalPoint.header.stamp = ros::Time::now();
            goalPoint.header.frame_id = "/map";
            goalPoint.ns = "turtlebotSlam";
            goalPoint.pose.orientation.w = 1.0;
            goalPoint.action = visualization_msgs::Marker::ADD;
            goalPoint.id = 1;
            goalPoint.type = visualization_msgs::Marker::POINTS;
            goalPoint.scale.x = mapResolution * 0.8f;
            goalPoint.scale.y = mapResolution * 0.8f;
            goalPoint.color.g = 1.0f;
            goalPoint.color.b = 1.0f;
            goalPoint.color.a = 1.0;

            geometry_msgs::Point p;
            p.x = (pose.x - mapSize[0]/2) * mapResolution - 7.5 * mapResolution;
            p.y = (pose.y - mapSize[1]/2) * mapResolution - 7.5 * mapResolution;
            //p.x = (destination.x - mapSize[0]/2) * mapResolution;
            //p.y= (destination.y - mapSize[1]/2) * mapResolution;
            p.z = 0.06;

            goalPoint.points.push_back(p);
            goalPoint.colors.push_back(goalPoint.color);
            goalPointPub.publish(goalPoint);

            ROS_ERROR("Published Goal Marker");
        }

        goalPub.publish(goal);
    }

    // Send a velocity command
    void move(double linearVelMPS, double angularVelRadPS) {
        geometry_msgs::Twist msg; // The default constructor will set all commands to 0
        msg.linear.x = linearVelMPS;
        msg.angular.z = angularVelRadPS;
        commandPub.publish(msg);
    }

    //Euclidean distance
    double distance(fd::_pose pose1,fd::_pose pose2){
        return sqrt(pow(pose1.x-pose2.x,2) + pow(pose1.y-pose2.y,2));
    }

    void spin() {
        ros::Rate rate(1); // Specify the FSM loop rate in Hz

        while (ros::ok()) {

            if(fsm == FSM_FOLLOW_GOAL){
                // Do nothing for now (just let navigation run)
            } if(fsm == FSM_SELECT_NEW_GOAL){
                selectNewDestination(); // Handles switching to different FSM State

                if(!targettingStack.empty()){
                    findPlanFor(destination.pos);
                } else {
                    fsm = FSM_RND_ROTATE;   // we have no targets left; switch to random
                }
            } else if(fsm == FSM_RND_MOVE_FORWARD) {
                move(FORWARD_SPEED_MPS,0);
            } else if(fsm == FSM_RND_ROTATE){
                if(ros::Time::now() > rotateStartTime+rotateDuration){
                    fsm=FSM_RND_MOVE_FORWARD;
                } else {
                    move(0,ROTATE_SPEED_RADPS);
                }
            }

            updateFrontiers();
            checkTargetValidity();

            ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
            rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
        }
    };

    enum FSM {FSM_RND_MOVE_FORWARD, FSM_RND_ROTATE, FSM_SELECT_NEW_GOAL, FSM_FOLLOW_GOAL};
    // Tunable parameters

    const static double MIN_SCAN_ANGLE_RAD = -20.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +20.0/180*M_PI;
    const static float PROXIMITY_RANGE_M = 0.75; // Should be smaller than sensor_msgs::LaserScan::range_max
    const static double FORWARD_SPEED_MPS = 0.1;
    const static double ROTATE_SPEED_RADPS = M_PI/8;


protected:
    ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
    ros::Publisher goalPub;
    ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
    ros::Subscriber mapSub;  //Subscriber to the gmapping map topic
    ros::Subscriber odomSub; //subscriber for the odom topic
    ros::Subscriber goalSub; //subscriber for the goal state
    ros::Publisher pointPub;
    ros::Publisher goalPointPub;
    ros::Timer timer;
    std::vector<fd::ValuePose> lastFoundFrontiers, targettingStack;
    boost::mt19937 rng;
    fd::ValuePose destination;
    _pose robot_pos;
    int mapSize[2];
    float mapResolution;
    enum FSM fsm; // Finite state machine for the random walk algorithm
    ros::Time rotateStartTime, ; // Start time of the rotation
    ros::Duration rotateDuration; // Duration of the rotation
    nav_msgs::OccupancyGrid::ConstPtr map;
    bool hasNewMap;
    bool changedDest, noPath;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    wfd::WaveFrontierDetector frontierDetector;
    // sensor measurements
    std::vector<_pose> sensorMeasurements;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "random_walk"); // Initiate new ROS node named "random_walk"
    ros::NodeHandle n;
    Controller walker(n); // Create new object
    walker.spin(); // Execute FSM loop
    return 0;
};
