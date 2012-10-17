#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void goalTopic(move_base_msgs::MoveBaseGoal goal) {
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the robot moved");
    else
      ROS_INFO("The robot failed to move for some reason");
}

int main(int argc, char** argv){

  ros::init(argc, argv, "frontier_navigation");
  ros::NodeHandle nh;
  ros::Subscriber goalSub = nh.subscribe("goal", 1, &goalTopic);

  ros::spin();
  return 0;
}
