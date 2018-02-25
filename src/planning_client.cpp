#include <ros/ros.h>
#include <phoebe_planning_msgs/PlanningRequest.h>
#include <phoebe_planning_msgs/MotionRequest.h>
#include <phoebe_planning_msgs/CollisionObject.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>

#include <geometry_msgs/Pose.h>

#include <eigen_conversions/eigen_msg.h>

class Client
{
private:
   ros::ServiceClient planning_request_service_;
   ros::ServiceClient motion_request_service_;
   ros::ServiceClient collision_object_service_;
   ros::Subscriber move_group_goal_;
   ros::Subscriber fake_controller_joint_states_;
   moveit::core::RobotModelPtr kinematic_model_;
   moveit::core::RobotStatePtr kinematic_state_;
   trajectory_msgs::JointTrajectory trajectory_;
public:
   Client(ros::NodeHandle nh)
   {
      ROS_INFO("Client()");
      planning_request_service_ = nh.serviceClient<phoebe_planning_msgs::PlanningRequest>("phoebe_planning_request");
      planning_request_service_.waitForExistence();
      motion_request_service_ = nh.serviceClient<phoebe_planning_msgs::MotionRequest>("phoebe_motion_request");
      motion_request_service_.waitForExistence();
      //collision_object_service_ = nh.serviceClient<phoebe_planning_msgs::CollisionObject>("phoebe_collision_object");
      move_group_goal_ = nh.subscribe("move_group/goal", 10, &Client::goalCallBack, this);
      fake_controller_joint_states_ = nh.subscribe("move_group/fake_controller_joint_states", 10, &Client::jointStatesCallBack, this);

      robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); 
      kinematic_model_ = robot_model_loader.getModel();
      kinematic_state_ = moveit::core::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
      kinematic_state_->setToDefaultValues(); 
   }

   void goalCallBack(const moveit_msgs::MoveGroupActionGoalConstPtr& msg)
   {
      ROS_INFO("should plan!");
      std::map<std::string, double> positions;
      for (unsigned int i = 0; i < msg->goal.request.goal_constraints[0].joint_constraints.size(); ++i)
      {
         positions.insert(std::pair<std::string, double>(msg->goal.request.goal_constraints[0].joint_constraints[i].joint_name, msg->goal.request.goal_constraints[0].joint_constraints[i].position));
      }
      kinematic_state_->setVariablePositions(positions);
      const Eigen::Affine3d &end_effector_state = kinematic_state_->getGlobalLinkTransform("right_arm_virtual_endeffector");
      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(end_effector_state, pose);
      ROS_WARN_STREAM("position: " << pose.position);
      ROS_WARN_STREAM("orientation: " << pose.orientation);
      geometry_msgs::PoseStamped posestamped;
      posestamped.header.frame_id = "/base_link";
      posestamped.pose = pose;
      phoebe_planning_msgs::PlanningRequest planning_request;
      planning_request.request.pose = posestamped;
      planning_request.request.id = 0;
      planning_request.request.duration = 16.0;
      planning_request_service_.call(planning_request);
      if (planning_request.response.result)
      {
         ROS_INFO("Plan successful!");
         trajectory_ = planning_request.response.trajectory;
      }
      else
      {
         trajectory_msgs::JointTrajectory trajectory;
         trajectory_ = trajectory;
      }
   }

   void jointStatesCallBack(const sensor_msgs::JointStateConstPtr& msg)
   {
      ROS_INFO("should execute!");
      if (trajectory_.points.size() > 0)
      {
         phoebe_planning_msgs::MotionRequest motion_request;
         motion_request.request.trajectory = trajectory_;
         motion_request.request.tolerance = 0.01;
         motion_request_service_.call(motion_request);
      }
   }
};

int main(int argc, char** argv)
{
   ROS_INFO("Phoebe MoveIt Planning client!");
   ros::init(argc, argv, "phoebe_planning_client");
   ros::NodeHandle nh;

   Client client(nh);
   
   ros::Rate r(10); // 10 Hz
   while (ros::ok())
   {
      ros::spinOnce();
      r.sleep();
   }

   return 0;
}
