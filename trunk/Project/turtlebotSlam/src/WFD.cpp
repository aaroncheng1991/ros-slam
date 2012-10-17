#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include <cstdlib> // Needed for rand()
#include <iostream>
#include <ctime> // Needed to seed random number generator with a time value
#include <cmath>
#include <ostream>
#include <fstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "turtlebotSlam/wavefrontierdetector.h"


class WFD {

public:
    // Construst a new RandomWalk object and hook up this ROS node
    // to the simulated robot's velocity control and laser topics
    WFD(ros::NodeHandle& nh) :
        fsm(FSM_MOVE_FORWARD),rotateStartTime(ros::Time::now()),rotateDuration(0.f) {
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
        laserSub = nh.subscribe("base_scan", 1, &WFD::commandCallback, this);
        mapSub = nh.subscribe("map", 1, &WFD::mapCallback, this);
        odomSub = nh.subscribe("odom", 1, &WFD::odomCallback, this);
        goalPub = nh.advertise<move_base_msgs::MoveBaseGoal>("goal", 1);
        hasMap = false;
    }

    // Send a velocity command
    void move(double linearVelMPS, double angularVelRadPS) {
        geometry_msgs::Twist msg; // The default constructor will set all commands to 0
        msg.linear.x = linearVelMPS;
        msg.angular.z = angularVelRadPS;
        //    commandPub.publish(msg);
    }


    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        try {
            // X and Y translation coordinate from the origin, where the robot started
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
            double turn = tf::getYaw(msg->pose.pose.orientation);

            robot_pos[0] = (mapSize[0] / 2) + ceil(x/mapResolution);
            robot_pos[1] = (mapSize[1] / 2) + ceil(y/mapResolution);
            robot_pos[2] = turn;
            //Print out current translated position of the robot


        } catch (tf::TransformException& ex) {
            ROS_ERROR("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
        }
        //ROS_ERROR("x: %f y: %f angle: %f", x, y, turn);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        this->map = msg;
        hasMap = true;
        mapSize[0] = msg->info.width;
        mapSize[1] = msg->info.height;
        mapResolution = msg->info.resolution;

        wfd::_pose pose;
        pose.x = robot_pos[0];
        pose.y = robot_pos[1];

        if(!hasMap || std::isnan(pose.x) || std::isnan(pose.y) || pose.x < 0 || pose.x >= map->info.width || pose.y < 0 || pose.y >= map->info.height) return;

        wfd::WaveFrontierDetector frontierDetector(map);
        std::vector<wfd::_pose> frontiers = frontierDetector.wfd(pose);

        double dist = DBL_MIN;
        ROS_ERROR("Distance between destination and robot: %f", distance(destination, pose));
        if(distance(destination, pose) < 0.1 || (destination.x==0 && destination.y==0)){
            ROS_ERROR("Setting changedDest to true");
            for(unsigned int i = 0; i < frontiers.size(); i++){
                double d = distance(frontiers[i], pose);
                if(d > dist && msg->data[frontiers[i].y*mapSize[0]+frontiers[i].x] !=-1){
                    dist = d;
                    destination = frontiers[i];
                }
            }
        }
         ROS_ERROR("Starting visualisation");
        // visualization
        visualization_msgs::Marker points;
        points.header.stamp = ros::Time::now();
        points.header.frame_id = "/map";
        points.ns = "turtlebotSlam";
        points.pose.orientation.w = 1.0;
        points.action = visualization_msgs::Marker::ADD;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.8;
        points.scale.y = 0.8;
        points.color.g = 1.0f;
        points.color.a = 0.01;
        for(unsigned int i = 0 ; i < frontiers.size() ; ++i) {
            geometry_msgs::Point p;
            p.x = (frontiers[i].x - mapSize[0]/2) * mapResolution;
            p.y= (frontiers[i].y - mapSize[1]/2) * mapResolution;
            p.z = 0;

            points.points.push_back(p);
            points.colors.push_back(points.color);
        }
         ROS_ERROR("First publish");
        pointPub.publish(points);
        visualization_msgs::Marker goalPoint;
        goalPoint.header.stamp = ros::Time::now();
        goalPoint.header.frame_id = "/map";
        goalPoint.ns = "turtlebotSlam";
        goalPoint.pose.orientation.w = 1.0;
        goalPoint.action = visualization_msgs::Marker::ADD;
        goalPoint.id = 0;
        goalPoint.type = visualization_msgs::Marker::POINTS;
        goalPoint.scale.x = 0.2;
        goalPoint.scale.y = 0.2;
        goalPoint.color.b = 1.0f;
        goalPoint.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = (destination.x - mapSize[0]/2) * mapResolution;
        p.y= (destination.y - mapSize[1]/2) * mapResolution;
        p.z = 0;

        ROS_ERROR("p.x: %f p.y: %f", p.x, p.y);

        goalPoint.points.push_back(p);
        goalPoint.colors.push_back(goalPoint.color);
        ROS_ERROR("second publish");
        goalPointPub.publish(goalPoint);
        ROS_ERROR("Setting new goal in life");
        simpleMove(destination);
    }

    void simpleMove(wfd::_pose pose){
        // publish the goal
        /*
geometry_msgs::TransformStamped geoTransform;
try {
    listener.lookupTransform("map",
                             "base_link",
                             ros::Time(0),
                             tfTransform);
}
catch(tf::TransformException &exception) {
    ROS_ERROR("%s", exception.what());
}

      geoTransform.transform.translation.x = tfTransform.getOrigin().x();
      geoTransform.transform.translation.y = tfTransform.getOrigin().y();
*/
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x=(pose.x - mapSize[0]/2) * mapResolution; //convergence from occupancygrid indexes to map coordinates
        goal.target_pose.pose.position.y=(pose.y - mapSize[1]/2) * mapResolution;
        goal.target_pose.pose.orientation.w = 1.0;
        goalPub.publish(goal);
        //      ac.waitForResult();
    }

    //Euclidean distance
    double distance(wfd::_pose pose1,wfd::_pose pose2){
        return sqrt(pow(pose1.x-pose2.x,2) + pow(pose1.y-pose2.y,2));
    }

    // Process the incoming laser scan message
    void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (fsm == FSM_MOVE_FORWARD) {

        }
    };

    void spin() {
        ros::Rate rate(5); // Specify the FSM loop rate in Hz
        while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
            if(fsm == FSM_MOVE_FORWARD) {
                move(FORWARD_SPEED_MPS,0);
            } else {
                if(ros::Time::now() > rotateStartTime+rotateDuration){
                    fsm=FSM_MOVE_FORWARD;
                }else{
                    move(0,ROTATE_SPEED_RADPS);
                }

            }
            ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
            rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
        }
    };

    enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
    // Tunable parameters

    const static double MIN_SCAN_ANGLE_RAD = -20.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +20.0/180*M_PI;
    const static float PROXIMITY_RANGE_M = 0.75; // Should be smaller than sensor_msgs::LaserScan::range_max
    const static double FORWARD_SPEED_MPS = 0.6;
    const static double ROTATE_SPEED_RADPS = M_PI/2;


protected:
    ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
    ros::Publisher goalPub;
    ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
    ros::Subscriber mapSub;  //Subscriber to the gmapping map topic
    ros::Subscriber odomSub; //subscriber for the odom topic
    ros::Publisher pointPub;
    ros::Publisher goalPointPub;
    tf::StampedTransform tfMap;
    tf::Vector3 origin;
    tf::Quaternion rotation;
    tf::TransformListener listener;
    tf::StampedTransform tfTransform;
    ros::Timer timer;
    wfd::_pose destination;
    double robot_pos[3];
    int mapSize[2];
    float mapResolution;
    enum FSM fsm; // Finite state machine for the random walk algorithm
    ros::Time rotateStartTime; // Start time of the rotation
    ros::Duration rotateDuration; // Duration of the rotation
    nav_msgs::OccupancyGrid::ConstPtr map;
    bool hasMap;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "random_walk"); // Initiate new ROS node named "random_walk"
    ros::NodeHandle n;
    WFD walker(n); // Create new object
    walker.spin(); // Execute FSM loop
    return 0;
};
