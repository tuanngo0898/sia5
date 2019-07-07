#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <urdf/model.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <std_msgs/Int8MultiArray.h>
#include <gazebo_msgs/SetModelState.h>

#include <string>

static const std::string PLANNING_GROUP = "arm";
moveit::planning_interface::MoveGroupInterface *move_group;
const robot_state::JointModelGroup* joint_model_group;
moveit_msgs::DisplayTrajectory display_trajectory;
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools *visual_tools;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

ros::Publisher gripper_pub_top_left;
ros::Publisher gripper_pub_top_right;
ros::Publisher gripper_pub_top_middle;

gazebo_msgs::SpawnModel::Request spawn_model_req;
gazebo_msgs::SpawnModel::Response spawn_model_resp;
ros::ServiceClient spawnClient;
std::string cylinder_path;
std::ifstream cylinder_inXml(cylinder_path.c_str());
std::stringstream cylinder_strStream;
std::string cylinder_xmlStr;

  //int to string converter
std::string intToString (int a) {
  std::stringstream ss;
  ss << a;
  return ss.str();
}

void grip_close(void){
  std_msgs::Float64 grip;
  grip.data = 0.35;
  gripper_pub_top_left.publish(grip);
  gripper_pub_top_right.publish(grip);
  gripper_pub_top_middle.publish(grip);
}

void grip_closer(void){
  std_msgs::Float64 grip;
  grip.data = 0.4;
  gripper_pub_top_left.publish(grip);
  gripper_pub_top_right.publish(grip);
  gripper_pub_top_middle.publish(grip);
}

void grip_open(void){
  std_msgs::Float64 grip;
  grip.data = 0;
  gripper_pub_top_left.publish(grip);
  gripper_pub_top_right.publish(grip);
  gripper_pub_top_middle.publish(grip);
}

int i =0;
void palm_pose_callback(const geometry_msgs::Pose& target_pose)
{
  ROS_INFO("palm_pose_callback");

  std::string index = intToString(i);
  i++;
  std::string model_name = "cylinder_" + index;;
  spawn_model_req.model_name = model_name;
  spawn_model_req.robot_namespace = model_name;
  spawn_model_req.model_xml = cylinder_xmlStr;
  spawn_model_req.reference_frame = "world";
  spawn_model_req.initial_pose.position = target_pose.position;
  spawn_model_req.initial_pose.orientation = target_pose.orientation;

  bool call_service = spawnClient.call(spawn_model_req, spawn_model_resp);
  if (call_service) {
    if (spawn_model_resp.success) {
      ROS_INFO_STREAM(model_name << " has been spawned");
    }
    else {
      ROS_INFO_STREAM(model_name << " spawn failed");
    }
  }
  else {
    ROS_INFO("fail in first call");
    ROS_ERROR("fail to connect with gazebo server");
    return;
  }

  bool success;
  geometry_msgs::Pose dst_pose;
  visual_tools->deleteAllMarkers();
  visual_tools->trigger();
  tf2::Quaternion orientation;

  grip_open();

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group->getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "object";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 0.3;
  primitive.dimensions[1] = 0.025;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 0;
  box_pose.position.x = target_pose.position.x;
  box_pose.position.y = target_pose.position.y-0.01;
  box_pose.position.z = target_pose.position.z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.addCollisionObjects(collision_objects);
  ROS_INFO("Add object into the world in Rviz");
  
  orientation.setRPY(-M_PI / 2, 0, -M_PI / 2);
  dst_pose = target_pose;
  // dst_pose.position.x -= 0.01;
  dst_pose.position.y += 0.04;
  dst_pose.position.z += 0.05;
  ROS_INFO("target posision %f %f %f", dst_pose.position.x, dst_pose.position.y, dst_pose.position.z);
  ROS_INFO("target orientation %f %f %f %f", dst_pose.orientation.x, dst_pose.orientation.y, dst_pose.orientation.z, dst_pose.orientation.w);
  dst_pose.orientation = tf2::toMsg(orientation);
  move_group->setPoseTarget(dst_pose);

  success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Planned");
  if(success){
    visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools->trigger();
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    move_group->execute(my_plan);
    ROS_INFO_NAMED("Move it controller", "Completed!");
  }
  else{
    ROS_INFO_NAMED("Move it controller", "Failed to plan!");
  }

  grip_close();
  ros::Duration(1).sleep();
  grip_closer();
  ros::Duration(2).sleep();

  // std::vector<moveit_msgs::Grasp> grasps;
  // grasps.resize(1);

  // grasps[0].grasp_pose.header.frame_id = "world";
  // orientation.setRPY(-M_PI / 2, 0, -M_PI / 2);
  // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  // grasps[0].grasp_pose.pose.position.x = target_pose.position.x;
  // grasps[0].grasp_pose.pose.position.y = target_pose.position.y+0.3;
  // grasps[0].grasp_pose.pose.position.z = target_pose.position.z;

  // /* Defined with respect to frame_id */
  // grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
  // /* Direction is set as positive x axis */
  // grasps[0].pre_grasp_approach.direction.vector.y = -0.5;
  // grasps[0].pre_grasp_approach.min_distance = 0.095;
  // grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // /* Defined with respect to frame_id */
  // grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
  // /* Direction is set as positive z axis */
  // grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  // grasps[0].post_grasp_retreat.min_distance = 0.1;
  // grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // move_group->pick("object", grasps);
  // grip_close();
  

  // std::vector<std::string> object_ids;
  // object_ids.push_back(std::string(collision_object.id));
  // planning_scene_interface.removeCollisionObjects(object_ids);
  // ROS_INFO("Remove object from the world in Rviz");

  // std::vector<geometry_msgs::Pose> waypoints;
  // geometry_msgs::Pose start_pose;
  // start_pose = move_group->getCurrentPose().pose;
  // ROS_INFO("Current position : %f %f %f", start_pose.position.x, start_pose.position.y, start_pose.position.z);

  // waypoints.push_back(start_pose);
  // start_pose.position.z += 0.1;
  // waypoints.push_back(start_pose);
  
  // move_group->setMaxVelocityScalingFactor(0.1);

  // moveit_msgs::RobotTrajectory trajectory;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  // double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // visual_tools->deleteAllMarkers();
  // visual_tools->publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  // for (std::size_t i = 0; i < waypoints.size(); ++i)
  // visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  // visual_tools->trigger();

  // geometry_msgs::Pose pose;
  // pose = move_group->getCurrentPose().pose;
  // pose.position.z += 0.1;
  // move_group->setPoseTarget(pose);
  // move_group->move();

  // orientation.setRPY(-M_PI / 2, 0, -M_PI / 2);
  // dst_pose = move_group->getCurrentPose().pose;
  // move_group->setPoseTarget(dst_pose);
  // move_group->setPoseTarget(target_pose);

  // ROS_INFO("Start plan");
  // success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO("Planned");
  // if(success){
  //   visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  //   visual_tools->trigger();

  //   ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

  //   // move_group->move();
  //   move_group->execute(my_plan);
  //   ROS_INFO_NAMED("Move it controller", "Completed!");
  // }
  // else{
  //   ROS_INFO_NAMED("Move it controller", "Failed to plan!");
  // }

  
  // dst_pose.orientation.x = 0.021;
  // dst_pose.orientation.y = 0.706;
  // dst_pose.orientation.z = -0.690;
  // dst_pose.orientation.w = -0.158;
  orientation.setRPY(-1.651, 0.084, 3.046);
  dst_pose.orientation = tf2::toMsg(orientation);
  dst_pose.position.x = -0.518;
  dst_pose.position.y = 0.490;
  dst_pose.position.z = 0.330;
  move_group->setPoseTarget(dst_pose);

  
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation = tf2::toMsg(orientation);
  // // target_pose1.orientation.w = 1.0;
  // target_pose1.position.x = 0.715;
  // target_pose1.position.y = 0.027;
  // target_pose1.position.z = 0.652;
  // move_group->setPoseTarget(target_pose1);

  success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools->trigger();

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

    // move_group->move();
    move_group->execute(my_plan);
    ROS_INFO_NAMED("Move it controller", "Completed!");
  }
  else{
    ROS_INFO_NAMED("Move it controller", "Failed to plan!");
  }

  grip_open();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_controller");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  bool get_cylinder_path;
  get_cylinder_path = node_handle.getParam("/cylinder_path", cylinder_path);
  if (!(get_cylinder_path)){
      return 0;}
  else{ROS_INFO_STREAM(cylinder_path << " has been extracted");}
  spawnClient = node_handle.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  std::ifstream cylinder_inXml(cylinder_path.c_str());
  cylinder_strStream << cylinder_inXml.rdbuf();
  cylinder_xmlStr = cylinder_strStream.str();

  ros::Subscriber sub = node_handle.subscribe("palm_pose", 1, palm_pose_callback);

  gripper_pub_top_left = node_handle.advertise<std_msgs::Float64>("/dhand/joint_finger_top_left_position_controller/command", 1);
  gripper_pub_top_right = node_handle.advertise<std_msgs::Float64>("/dhand/joint_finger_top_right_position_controller/command", 1);
  gripper_pub_top_middle = node_handle.advertise<std_msgs::Float64>("/dhand/joint_finger_top_middle_position_controller/command", 1);

  grip_open();

  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  visual_tools = new moveit_visual_tools::MoveItVisualTools("world");
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  visual_tools->deleteAllMarkers();
  visual_tools->loadRemoteControl();

  ROS_INFO("Reference frame: %s", move_group->getPlanningFrame().c_str());
  move_group->setEndEffectorLink("dhand_palmt_link");
  ROS_INFO("End Effector Link: %s", move_group->getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("Move it controller", "Available Planning Groups:");
  std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  ROS_INFO_NAMED("Move it controller", "\r\n");

  // std::vector<moveit_msgs::PlaceLocation> place_location;
  // place_location.resize(1);

  // place_location[0].place_pose.header.frame_id = "world";
  // 
  // orientation.setRPY(0, 0, M_PI / 2);
  // place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  // /* While placing it is the exact location of the center of the object. */
  // place_location[0].place_pose.pose.position.x = 0;
  // place_location[0].place_pose.pose.position.y = 0.5;
  // place_location[0].place_pose.pose.position.z = 0.5;

  // /* Defined with respect to frame_id */
  // place_location[0].pre_place_approach.direction.header.frame_id = "world";
  // /* Direction is set as negative z axis */
  // place_location[0].pre_place_approach.direction.vector.z = -1.0;
  // place_location[0].pre_place_approach.min_distance = 0.095;
  // place_location[0].pre_place_approach.desired_distance = 0.115;

  // /* Defined with respect to frame_id */
  // place_location[0].post_place_retreat.direction.header.frame_id = "world";
  // /* Direction is set as negative y axis */
  // place_location[0].post_place_retreat.direction.vector.y = -1.0;
  // place_location[0].post_place_retreat.min_distance = 0.1;
  // place_location[0].post_place_retreat.desired_distance = 0.25;

  // move_group->place("object", place_location);

  ros::waitForShutdown();
  return 0;
}
