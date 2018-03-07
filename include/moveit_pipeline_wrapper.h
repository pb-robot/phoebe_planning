
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <actionlib/client/simple_action_client.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <geometric_shapes/shape_operations.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <pluginlib/class_loader.h>
#include <tf/transform_listener.h>
#include <dynamixel_msgs/JointState.h>
#include <vector>

class MoveItPlanner
{
   private:
      //ros::Publisher collision_object_publisher;
      moveit_msgs::WorkspaceParameters ws;
      std::vector<double> joint_states;

      planning_scene::PlanningScenePtr planning_scene;
      planning_interface::PlannerManagerPtr planner_instance;

      robot_model_loader::RobotModelLoader robot_model_loader;
      robot_model::RobotModelPtr robot_model;

      boost::shared_ptr<tf::TransformListener> tf;
      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;

      boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
      tf::TransformListener listener;

      ros::NodeHandle nodehandle_;

      ros::Publisher display_publisher_;
      std::vector<ros::Subscriber> subscribers_;

   public:
      MoveItPlanner(ros::NodeHandle n, std::string);
      void trajectory_fix(int index, trajectory_msgs::JointTrajectory* trajectory,
            trajectory_msgs::JointTrajectory* new_trajectory);
      bool motion_plan(geometry_msgs::Pose goal, trajectory_msgs::JointTrajectory* solution_trajectory, double time_limit);
      void update_joints(std::vector<double> joints);
      void add_workpiece_collisionobject(geometry_msgs::Pose pose);
      bool nonblocking_move(geometry_msgs::Pose goal, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac, int time_limit);
      void update_human_collisionobject(geometry_msgs::Pose pose);
      void attach_object();
      void jointStateMessage(const dynamixel_msgs::JointStateConstPtr& message, const int id);
      std::vector<float> extractFinalPose(trajectory_msgs::JointTrajectory trajectory);
};
