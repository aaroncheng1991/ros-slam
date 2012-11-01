#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/Int32.h"
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher goalStatePub;

void goalTopic(move_base_msgs::MoveBaseGoal goal) {
    ROS_ERROR("Frontier Navigation received request to move");

    std_msgs::Int32 startMsg;
    startMsg.data = 0;
    goalStatePub.publish(startMsg);

    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ac.sendGoal(goal);

    ac.waitForResult();

    std_msgs::Int32 doneMsg;

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the robot moved");
        doneMsg.data = 1;
    } else {
        ROS_INFO("The robot failed to move for some reason");
        doneMsg.data = 1;
    }
    goalStatePub.publish(doneMsg);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "frontier_navigation");
    ros::NodeHandle nh;
    ros::Subscriber goalSub = nh.subscribe("goal", 1, &goalTopic);
    goalStatePub = nh.advertise<std_msgs::Int32>("goalState", 1);

    ros::spin();
    return 0;
}
