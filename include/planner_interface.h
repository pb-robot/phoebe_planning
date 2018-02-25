#include <geometry_msgs/PoseStamped.h>
#include "moveit_pipeline_wrapper.h"
#include <phoebe_planning_msgs/PlanningRequest.h>
#include <phoebe_planning_msgs/MotionRequest.h>
#include <phoebe_planning_msgs/GripperRequest.h>

namespace phoebe
{
   class PlannerInterface
   {
      private:
         MoveItPlanner* planner_;
         actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac_;
         ros::ServiceServer planning_service_;
         ros::ServiceServer motion_service_;
         ros::ServiceServer gripper_service_;
         ros::Publisher gripper_angle_right_;
      public:
         PlannerInterface(ros::NodeHandle, std::string, std::string, std::string, std::string);
         //void manipulatorPlannerCallback(const geometry_msgs::PoseStampedConstPtr pose);
         bool planRequestServiceCall(phoebe_planning_msgs::PlanningRequest::Request &request, phoebe_planning_msgs::PlanningRequest::Response &response);
         bool motionRequestServiceCall(phoebe_planning_msgs::MotionRequest::Request &request, phoebe_planning_msgs::MotionRequest::Response &response);
         bool gripperMotionRequestServiceCall(phoebe_planning_msgs::GripperRequest::Request &request, phoebe_planning_msgs::GripperRequest::Response &response);
   };
}
