#include <ros/ros.h>
#include "planner_interface.h"


int main(int argc, char** argv)
{
   ROS_INFO("Phoebe MoveIt Planning");
   ros::init(argc, argv, "phoebe_planning");
   ros::NodeHandle nh;
   phoebe::PlannerInterface planner(nh, "robot_description", "phoebe_planning_request", "phoebe_motion_request", "phoebe_gripper_request");
   ros::spin();
   return 0;
}
