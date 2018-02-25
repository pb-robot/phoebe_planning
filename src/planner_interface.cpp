#include "planner_interface.h"
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float64.h>

namespace phoebe
{
   PlannerInterface::PlannerInterface(ros::NodeHandle nh, std::string robot_description, std::string plan_request, std::string motion_request, std::string gripper_request)
   {
      ROS_INFO("PlannerInterface()");
      planner_ = new MoveItPlanner(nh, robot_description);
      ac_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("right_arm_controller/follow_joint_trajectory", true);
      ROS_INFO("Waiting for action server...");
      ac_->waitForServer();
      planning_service_ = nh.advertiseService(plan_request, &PlannerInterface::planRequestServiceCall, this);
      motion_service_ = nh.advertiseService(motion_request, &PlannerInterface::motionRequestServiceCall, this);
      gripper_service_ = nh.advertiseService(gripper_request, &PlannerInterface::gripperMotionRequestServiceCall, this);
      gripper_angle_right_ = nh.advertise<std_msgs::Float64>("/right_arm_gripper_joint/command", 10);
      ROS_INFO("Planning service running!");
   }
   
   bool PlannerInterface::gripperMotionRequestServiceCall(phoebe_planning_msgs::GripperRequest::Request &request, phoebe_planning_msgs::GripperRequest::Response &response)
   {
      ROS_INFO("Gripper motion request received!");
      // -0.08 .. 0.08
      if (request.angle > -0.5 && request.angle < 0.5)
      {
         std_msgs::Float64 msg;
         msg.data = request.angle;
         gripper_angle_right_.publish(msg);
         ros::Duration(request.duration).sleep();
         response.result = true;
         return true;
      }
      else
      {
         ROS_ERROR("Incorrect angle! Angle must be between -0.5 and 0.5");
         response.result = false;
         return false;
      }
   }

   bool PlannerInterface::motionRequestServiceCall(phoebe_planning_msgs::MotionRequest::Request &request, phoebe_planning_msgs::MotionRequest::Response &response)
   {
      ROS_INFO("Motion request received!");
      if (request.trajectory.joint_names.size() == 0)
      {
         ROS_ERROR("Trajectory error (joint_names array empty!)!");
         return false;
      }

      control_msgs::FollowJointTrajectoryGoal fojotrag;
      ac_->cancelAllGoals();

      fojotrag.trajectory = request.trajectory;
      fojotrag.path_tolerance.clear();
      fojotrag.goal_tolerance.clear();
      fojotrag.goal_time_tolerance = ros::Duration(1);
      for (unsigned int j = 0; j < fojotrag.trajectory.joint_names.size(); ++j)
      {
         control_msgs::JointTolerance jtol;
         control_msgs::JointTolerance gtol;
         jtol.name = fojotrag.trajectory.joint_names[j];
         gtol.name = fojotrag.trajectory.joint_names[j];
         jtol.position = 6.1;
         jtol.velocity = 60.1;
         jtol.acceleration = 60.1;
         gtol.position = 0.001;
         gtol.velocity = 0.0;
         gtol.acceleration = 0.0;
         fojotrag.path_tolerance.push_back(jtol);
         fojotrag.goal_tolerance.push_back(gtol);
      }

      ac_->sendGoal(fojotrag);
      ros::Duration(request.trajectory.points[request.trajectory.points.size()-1].time_from_start).sleep();
      response.result = true;
      response.joint_angles = planner_->extractFinalPose(request.trajectory);
      return true;
   }

   bool PlannerInterface::planRequestServiceCall(phoebe_planning_msgs::PlanningRequest::Request &request, phoebe_planning_msgs::PlanningRequest::Response &response)
   {
      ROS_INFO("Planning request received!");
      trajectory_msgs::JointTrajectory solution_trajectory;
      try {
         if(planner_->motion_plan(request.pose.pose, &solution_trajectory, request.duration))
         {
            ROS_WARN("Successful plan!");
            response.result = true;
            response.joint_angles = planner_->extractFinalPose(solution_trajectory);
         }
         else
         {
            response.result = false;
            response.joint_angles = std::vector<float>();
         }
         response.trajectory = solution_trajectory;
      }
      catch (std::exception e)
      {
         ROS_ERROR("Planning failed!");
         response.result = false;
      }
      return true;
   }

}
