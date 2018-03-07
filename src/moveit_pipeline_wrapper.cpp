#include "moveit_pipeline_wrapper.h"

void MoveItPlanner::jointStateMessage(const dynamixel_msgs::JointStateConstPtr& message, const int id)
{
   joint_states[id] = message->current_pos;
}

MoveItPlanner::MoveItPlanner(ros::NodeHandle n, std::string robot_description) : nodehandle_(n)
{
   std::string topics[18] = {"right_arm_shoulder_roll_joint", "right_arm_shoulder_pitch_joint", "right_arm_shoulder_yaw_joint", "right_arm_elbow_pitch_joint", "right_arm_elbow_yaw_joint", "right_arm_wrist_pitch_joint", "right_arm_wrist_roll_joint", "right_arm_gripper_joint", "right_arm_gripper_joint2", "left_arm_shoulder_roll_joint", "left_arm_shoulder_pitch_joint", "left_arm_shoulder_yaw_joint", "left_arm_elbow_pitch_joint", "left_arm_elbow_yaw_joint", "left_arm_wrist_pitch_joint", "left_arm_wrist_roll_joint", "left_arm_gripper_joint", "left_arm_gripper_joint2"};
   for (unsigned int i = 0; i < 18; ++i)
   {
      joint_states.push_back(0.0);
      subscribers_.push_back(nodehandle_.subscribe<dynamixel_msgs::JointState>(topics[i] + "/state", 1, boost::bind(&MoveItPlanner::jointStateMessage, this, _1, i)));
   }
   display_publisher_ = nodehandle_.advertise<moveit_msgs::DisplayTrajectory>("/phoebe_move_group/display_planned_path", 1, true);
   //collision_object_publisher = n.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
   /* Load the robot model */
   robot_model_loader = robot_model_loader::RobotModelLoader(robot_description); // moveit_robot_description
   /* Get a shared pointer to the model and construct a state */
   robot_model = robot_model_loader.getModel();


   robot_state::RobotState current_state = robot_state::RobotState(robot_model);

   if (current_state.getVariableCount() == 0)
   {
      ROS_ERROR("MoveIt! parameters not loaded - run 'roslaunch tut params.launch'");
      return;
   }
   planning_scene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model));
   // add a box to the scene as collidable object
   //tf = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(ros::Duration(2.0)));
   //planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor("moveit_robot_description", tf));
   /*ros::Publisher attached_object_publisher = n.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
     while(attached_object_publisher.getNumSubscribers() < 1)
     {
     ROS_ERROR("sleepy sleepy");
     ros::WallDuration sleep_t(0.5);
     sleep_t.sleep();
     }*/


   std::string planner_plugin_name;

   //if (!n.getParam("planning_plugin", planner_plugin_name))
   //    ROS_FATAL_STREAM("Could not find planner plugin name");
   planner_plugin_name = "ompl_interface/OMPLPlanner";
   try
   {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
   }
   catch(pluginlib::PluginlibException& ex)
   {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
   }
   try
   {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, n.getNamespace()))
         ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
   }
   catch(pluginlib::PluginlibException& ex)
   {
      const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0 ; i < classes.size() ; ++i)
         ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
            << "Available plugins: " << ss.str());
   }
   ws.header.frame_id = "base_link";
   geometry_msgs::Vector3 a;
   geometry_msgs::Vector3 b;
   a.x = -1;
   a.y = -1;
   a.z = -1;
   b.x = 1;
   b.y = 1;
   b.z = 1;
   ws.min_corner = a;
   ws.max_corner = b;

   tf = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(ros::Duration(2.0)));
   planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor(planning_scene, robot_description, tf));
   planning_scene_monitor->startSceneMonitor();

   //    moveit_msgs::PlanningScene planning_scene_msg;
   //    moveit_msgs::CollisionObject collision_object;
   //    //collision_object.link_name = "table_link";
   //    /* The header must contain a valid TF frame*/
   //    collision_object.header.frame_id = "/table_link";
   //    /* The id of the object */
   //
   //    /* A default pose */
   //    geometry_msgs::Pose pose;
   //    shape_msgs::SolidPrimitive primitive;
   //    
   //    collision_object.id = "disptop";
   //    pose.orientation.w = 1;
   //    pose.orientation.x = 0;
   //    pose.orientation.y = 0;
   //    pose.orientation.z = 0;
   //    pose.position.x = -0.6;
   //    pose.position.y = 0.7;
   //    pose.position.z = 0.3;
   //    primitive.type = primitive.BOX;
   //    primitive.dimensions.resize(3);
   //    primitive.dimensions[0] = 0.40;
   //    primitive.dimensions[1] = 0.40;
   //    primitive.dimensions[2] = 0.02;
   //    collision_object.primitives.push_back(primitive);
   //    collision_object.primitive_poses.push_back(pose);
   //    collision_object.operation = collision_object.ADD;
   //    planning_scene_msg.world.collision_objects.push_back(collision_object);
   //
   //    collision_object.primitives.clear();
   //    collision_object.primitive_poses.clear();
   //    collision_object.id = "table";
   //    pose.orientation.w = 1;
   //    pose.orientation.x = 0;
   //    pose.orientation.y = 0;
   //    pose.orientation.z = 0;
   //    pose.position.x = -0.1;
   //    pose.position.y = 0.1;
   //    pose.position.z = -0.1;
   //    primitive.type = primitive.BOX;
   //    primitive.dimensions.resize(3);
   //    primitive.dimensions[0] = 2.40;
   //    primitive.dimensions[1] = 2.40;
   //    primitive.dimensions[2] = 0.1;
   //    collision_object.primitives.push_back(primitive);
   //    collision_object.primitive_poses.push_back(pose);
   //    collision_object.operation = collision_object.ADD;
   //    planning_scene_msg.world.collision_objects.push_back(collision_object);
   //
   //    collision_object.primitives.clear();
   //    collision_object.primitive_poses.clear();
   //    collision_object.id = "assistent";
   //    pose.orientation.w = 1;
   //    pose.orientation.x = 0;
   //    pose.orientation.y = 0;
   //    pose.orientation.z = 0;
   //    pose.position.x = -11.1;
   //    pose.position.y = 11.1;
   //    pose.position.z = -11.1;
   //    primitive.type = primitive.BOX;
   //    primitive.dimensions.resize(3);
   //    primitive.dimensions[0] = 0.40;
   //    primitive.dimensions[1] = 0.40;
   //    primitive.dimensions[2] = 2.1;
   //    collision_object.primitives.push_back(primitive);
   //    collision_object.primitive_poses.push_back(pose);
   //    collision_object.operation = collision_object.ADD;
   //    planning_scene_msg.world.collision_objects.push_back(collision_object);
   //
   //    collision_object.primitives.clear();
   //    collision_object.primitive_poses.clear();
   //    collision_object.id = "patient";
   //    pose.orientation.w = 1;
   //    pose.orientation.x = 0;
   //    pose.orientation.y = 0;
   //    pose.orientation.z = 0;
   //    pose.position.x = 0.4;
   //    pose.position.y = 0.0;
   //    pose.position.z = 0.2;
   //    primitive.type = primitive.BOX;
   //    primitive.dimensions.resize(3);
   //    primitive.dimensions[0] = 0.30;
   //    primitive.dimensions[1] = 2.40;
   //    primitive.dimensions[2] = 0.1;
   //    collision_object.primitives.push_back(primitive);
   //    collision_object.primitive_poses.push_back(pose);
   //    collision_object.operation = collision_object.ADD;
   //    planning_scene_msg.world.collision_objects.push_back(collision_object);
   //
   //    planning_scene->setPlanningSceneDiffMsg(planning_scene_msg);
   //
   //    /*
   //    shapes::Mesh* mesh = new shapes::Mesh();
   //    mesh = shapes::createMeshFromResource("package://ipa325_robot_descriptions/meshes/human/edited-Human_right_arm.dae", Eigen::Vector3d(1.2, 1.2, 1.2));
   //    ROS_INFO("mesh triangle_count = %d", mesh->triangle_count);
   //
   //    collision_object.id = "assistent";
   //    pose.orientation.w = 0.7071;
   //    pose.orientation.x = 0;
   //    pose.orientation.y = 0;
   //    pose.orientation.z = -0.7071;
   //    pose.position.x = -0.5;
   //    pose.position.y = 0.38;
   //    pose.position.z = 100.6;
   //    collision_object.operation = collision_object.ADD;
   //
   //    collision_object.primitives.clear();
   //    collision_object.primitive_poses.clear();
   //    shapes::ShapeMsg shapemsg;
   //    shapes::constructMsgFromShape(mesh, shapemsg);
   //
   //    collision_object.meshes.push_back(boost::get<shape_msgs::Mesh>(shapemsg));
   //    collision_object.mesh_poses.push_back(pose);
   //
   //
   //    planning_scene_msg.world.collision_objects.push_back(collision_object);
   //    planning_scene->setPlanningSceneDiffMsg(planning_scene_msg);*/
   //
   //    /*planning_scene_msg.is_diff = true;
   //    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
   //    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
   //    {
   //        ROS_ERROR("waiting for planning scene monitor to subscribe to planning_scene topic");
   //        ros::WallDuration sleep_t(0.5);
   //        sleep_t.sleep();    
   //    }
   //    planning_scene_diff_publisher.publish(planning_scene_msg);
   //    planning_scene->setPlanningSceneDiffMsg(planning_scene_msg);
   //    ROS_WARN("collision object added");*/

}

void MoveItPlanner::add_workpiece_collisionobject(geometry_msgs::Pose pose)
{

   moveit_msgs::PlanningScene planning_scene_msg;
   moveit_msgs::CollisionObject collision_object;
   //collision_object.link_name = "table_link";
   /* The header must contain a valid TF frame*/
   collision_object.header.frame_id = "/table_link";
   /* The id of the object */
   collision_object.id = "box";

   /* A default pose */
   geometry_msgs::Pose primpose;

   /* Define a box to be attached */
   shape_msgs::SolidPrimitive primitive;
   //shape_msgs::Mesh mesh;
   primitive.type = primitive.BOX;
   primitive.dimensions.resize(3);
   primitive.dimensions[0] = 0.05;
   primitive.dimensions[1] = 0.4;
   primitive.dimensions[2] = 0.13;
   collision_object.primitives.push_back(primitive);
   /*for (unsigned int i = 0; i < 14; ++i)
     {
     geometry_msgs::Point vertex;
     vertex.x = 1;
     vertex.y = 2;
     vertex.z = 3;
     mesh.vertices.push_back(vertex);
     }
     for (unsigned int i = 0; i < N; ++i)
     {
     shape_msgs::MeshTriangle triangle;
     triangle.vertex_indices[0] = 0;
     triangle.vertex_indices[1] = 1;
     triangle.vertex_indices[2] = 2;
     mesh.triangles.push_back(triangle);
     }*/


   // collision_object.meshes.push_back(mesh);
   primpose.orientation.w = 1.0;
   primpose.position.x = 0.25;
   primpose.position.y = 0;
   primpose.position.z = 0.22;
   collision_object.primitive_poses.push_back(primpose);
   collision_object.operation = collision_object.ADD;
   planning_scene_msg.world.collision_objects.push_back(collision_object);

   planning_scene_msg.is_diff = true;
   //ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
   /*while(planning_scene_diff_publisher.getNumSubscribers() < 1)
     {
     ROS_ERROR("waiting for planning scene monitor to subscribe to planning_scene topic");
     ros::WallDuration sleep_t(0.5);
     sleep_t.sleep();    
     }
     planning_scene_diff_publisher.publish(planning_scene_msg);*/

}

void MoveItPlanner::update_human_collisionobject(geometry_msgs::Pose pose)
{

   moveit_msgs::PlanningScene planning_scene_msg;
   moveit_msgs::CollisionObject collision_object;

   collision_object.header.frame_id = "/table_link";
   /* The id of the object */
   collision_object.id = "assistent";
   collision_object.operation = collision_object.MOVE;

   //collision_object.mesh_poses.push_back(pose);
   collision_object.primitive_poses.push_back(pose);

   planning_scene_msg.world.collision_objects.push_back(collision_object);
   planning_scene->setPlanningSceneDiffMsg(planning_scene_msg);

}

void update_workpiece_collisionobject(geometry_msgs::Pose pose)
{

}

void MoveItPlanner::update_joints(std::vector<double> joints)
{
   for (unsigned int i = 0; i < joint_states.size(); ++i)
      joint_states[i] = joints[i];
}

void MoveItPlanner::attach_object()
{
   ros::Publisher attached_object_publisher = nodehandle_.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
   while(attached_object_publisher.getNumSubscribers() < 1)
   {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();    
   }
   moveit_msgs::AttachedCollisionObject attached_object;
   attached_object.link_name = "r_wrist_roll_link";
   attached_object.object.header.frame_id = "r_wrist_roll_link";
   attached_object.object.id = "box";
   geometry_msgs::Pose pose;
   pose.orientation.w = 1.0;
   shape_msgs::SolidPrimitive primitive;
   primitive.type = primitive.BOX;  
   primitive.dimensions.resize(3);
   primitive.dimensions[0] = 0.1;
   primitive.dimensions[1] = 0.1;
   primitive.dimensions[2] = 0.1;  
   attached_object.object.primitives.push_back(primitive);
   attached_object.object.primitive_poses.push_back(pose);
   attached_object.object.operation = attached_object.object.ADD;
   attached_object_publisher.publish(attached_object); 
}

bool MoveItPlanner::motion_plan(geometry_msgs::Pose goal, trajectory_msgs::JointTrajectory* solution_trajectory, double time_limit)
{
   //ROS_ERROR("collision = %s", planning_scene->getActiveCollisionDetectorName().c_str());
   //planning_scene->printKnownObjects(std::cout);
   // define starting state
   ROS_ERROR("motion_plan(%.3f %.3f %.3f)", goal.position.x, goal.position.y, goal.position.z);
   moveit_msgs::RobotState startstate;
   std::string jnames[7] = {"right_arm_shoulder_roll_joint", "right_arm_shoulder_pitch_joint", "right_arm_shoulder_yaw_joint", "right_arm_elbow_pitch_joint", "right_arm_elbow_yaw_joint", "right_arm_wrist_pitch_joint", "right_arm_wrist_roll_joint"};
   for (unsigned int i = 0; i < 7; ++i)
   {
      startstate.joint_state.name.push_back(jnames[i]);
      /*if (i == 5)
        {
        if (joint_states[i] > 3.141)
        startstate.joint_state.position.push_back(-3.141);
        else if (joint_states[i] < -3.141)
        startstate.joint_state.position.push_back(3.141);
        else
        startstate.joint_state.position.push_back(joint_states[i]);
        }
        else*/
      startstate.joint_state.position.push_back(joint_states[i]);

   }

   planning_scene->setCurrentState(startstate);

   //Eigen::Affine3d tool = current_state.getGlobalLinkTransform("tool_link_c");
   //Eigen::Affine3d tcp = current_state.getGlobalLinkTransform("tcp");
   //Eigen::Quaterniond tool_q(tool.rotation());
   //Eigen::Quaterniond tcp_q(tcp.rotation());
   //Eigen::Vector3d ee_offset(-0.002316, 0.0079, 0.079425);
   Eigen::Vector3d ee_offset(0, 0, 0);

   Eigen::Quaterniond goal_q = Eigen::Quaterniond(goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z); // * Eigen::Quaterniond(1, 0, 0, 0);
   //ee_offset = goal_q * ee_offset;

   planning_interface::MotionPlanRequest req;
   planning_interface::MotionPlanResponse res;
   geometry_msgs::PoseStamped pose;
   pose.header.frame_id = "base_link";
   pose.pose.position.x = goal.position.x - ee_offset.x();
   pose.pose.position.y = goal.position.y - ee_offset.y();
   pose.pose.position.z = goal.position.z - ee_offset.z();

   pose.pose.orientation.w = goal_q.w();
   pose.pose.orientation.x = goal_q.x();
   pose.pose.orientation.y = goal_q.y();
   pose.pose.orientation.z = goal_q.z();

   /*Eigen::Matrix3d rm = Eigen::Quaterniond(0.0, 0.7071, 0, 0).toRotationMatrix();
     for (unsigned int i = 0; i < 3; ++i)
     {
     Eigen::Vector3d row = rm.row(i);
     for (unsigned int j = 0; j < 3; ++j)
     ROS_INFO("%d %d %.2f", i, j, row[j]);
     }*/

   double tolerance_pose = 0.0001;



   req.planner_id = "RRTstarkConfigDefault";
   req.num_planning_attempts = 2;
   req.allowed_planning_time = 0.5;
   req.group_name = "right_arm";
   req.workspace_parameters = ws;
   moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("right_arm_virtual_endeffector", pose, tolerance_pose, tolerance_pose);
   req.goal_constraints.push_back(pose_goal);
   planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
   context->solve(res);
   if(res.error_code_.val != res.error_code_.SUCCESS)
   {
      ROS_ERROR("Could not compute plan successfully");
      return false;
   }

   moveit_msgs::MotionPlanResponse response;
   res.getMessage(response);
   moveit_msgs::DisplayTrajectory display_trajectory;
   display_trajectory.trajectory_start = response.trajectory_start;

   //response.trajectory.joint_trajectory.joint_names.erase(response.trajectory.joint_trajectory.joint_names.end()-1);
   for (unsigned int i = 0; i < response.trajectory.joint_trajectory.points.size(); ++i)
   {
      response.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(((double)i+1)*(time_limit/((double)response.trajectory.joint_trajectory.points.size())));
      //ROS_INFO("deleting %.8f", response.trajectory.joint_trajectory.points[i].positions[6]);
      //response.trajectory.joint_trajectory.points[i].positions.erase(response.trajectory.joint_trajectory.points[i].positions.end()-1);
   }
   *solution_trajectory = response.trajectory.joint_trajectory;

   display_trajectory.trajectory.push_back(response.trajectory);
   display_publisher_.publish(display_trajectory);

   //for (unsigned int i = 0; i < joint_states.size(); ++i)
   //   joint_states[i] = response.trajectory.joint_trajectory.points[response.trajectory.joint_trajectory.points.size()-1].positions[i];

   return true;
}

void MoveItPlanner::trajectory_fix(int index, trajectory_msgs::JointTrajectory* trajectory,
      trajectory_msgs::JointTrajectory* new_trajectory)
{
   /*
    * Merges and compresses trajectory.points.positions[index] with new_trajectory points
    * (e.g., merges end-effector motion into manipulator motion)
    */
   double delta = new_trajectory->points[new_trajectory->points.size()-1].positions[index] - trajectory->points[0].positions[index];
   ROS_WARN("EE start = %.3f   end = %.3f  delta = %.3f", new_trajectory->points[new_trajectory->points.size()-1].positions[index], trajectory->points[0].positions[index], delta);
   for (unsigned int i = 1; i < trajectory->points.size(); ++i)
      trajectory->points[i].positions[index] = trajectory->points[0].positions[index] + (delta/(trajectory->points.size()-1))*i;
}

std::vector<float> MoveItPlanner::extractFinalPose(trajectory_msgs::JointTrajectory trajectory)
{
   std::vector<float> return_value;
   for (unsigned int i = 0; i < trajectory.points[trajectory.points.size()-1].positions.size(); ++i)
   {
      return_value.push_back(trajectory.points[trajectory.points.size()-1].positions[i]);
   }
   return return_value;
}

bool MoveItPlanner::nonblocking_move(geometry_msgs::Pose goal, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* ac, int time_limit)
{
   //    tf::StampedTransform transform;
   //    listener.lookupTransform("/table_link", "c_human_arm_r_link", ros::Time(0), transform);
   //    geometry_msgs::Pose pose;
   //    pose.position.x = transform.getOrigin()[0];
   //    pose.position.y = transform.getOrigin()[1];
   //    pose.position.z = transform.getOrigin()[2];
   //    //pose.position.z = 1.5;
   //    Eigen::Quaterniond q(transform.getRotation());
   //    pose.orientation.w = q.w();
   //    pose.orientation.x = q.x();
   //    pose.orientation.y = q.y();
   //    pose.orientation.z = q.z();
   //    update_human_collisionobject(pose);
   //
   //    planning_scene->saveGeometryToStream(std::cout);

   trajectory_msgs::JointTrajectory* jt(new trajectory_msgs::JointTrajectory);


   control_msgs::FollowJointTrajectoryGoal fojotrag;
   ac->cancelAllGoals();

   if (!motion_plan(goal, jt, time_limit)) return false; // main motion
   trajectory_msgs::JointTrajectory* eet(new trajectory_msgs::JointTrajectory);
   for (unsigned int i = 0; i < 8; ++i)
   {
      if (!motion_plan(goal, eet, 16)) return false; // end effector
   }
   //trajectory_fix(5, jt, eet);

   for (unsigned int i = 0; i < 1; ++i) // TODO horrible hacks all over
   {
      //ac->stopTrackingGoal();
      //if (!moveit_motion_plan(goal, jt, 16)) return false;
      fojotrag.trajectory = *jt;
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
      //ROS_ERROR("sendgoalandwait");
      ac->sendGoal(fojotrag);
      //ROS_ERROR("getResult() == %d", (*ac->getResult()).error_code);
   }

   /*ros::Duration d(10);
     d.sleep();
     ROS_ERROR("VALID == %s", planning_scene->isStateValid(planning_scene->getCurrentState()) ? "TRUE" : "FALSE");*/

   return true;
}
