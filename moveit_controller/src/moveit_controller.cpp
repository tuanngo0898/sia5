#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static const std::string PLANNING_GROUP = "arm";
moveit::planning_interface::MoveGroupInterface *move_group;
const robot_state::JointModelGroup* joint_model_group;
moveit_msgs::DisplayTrajectory display_trajectory;
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools *visual_tools;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
std::vector<moveit_msgs::CollisionObject> collision_objects;

void palm_pose_callback(const geometry_msgs::Pose& target_pose)
{

  ROS_INFO("palm_pose_callback");
  
  visual_tools->deleteAllMarkers();

  collision_objects[1].id = "object";
  collision_objects[1].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;

  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "world";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  move_group->pick("object", grasps);

  move_group->setPoseTarget(target_pose);
  ROS_INFO("Start plan");
  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Planned");
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

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_controller");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Subscriber sub = node_handle.subscribe("palm_pose", 10, palm_pose_callback);

  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  visual_tools = new moveit_visual_tools::MoveItVisualTools("world");
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  visual_tools->deleteAllMarkers();
  visual_tools->loadRemoteControl();

  ROS_INFO("Reference frame: %s", move_group->getPlanningFrame().c_str());
  move_group->setEndEffectorLink("dhand_palm_link");
  ROS_INFO("End Effector Link: %s", move_group->getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("Move it controller", "Available Planning Groups:");
  std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  ROS_INFO_NAMED("Move it controller", "\r\n");


  collision_objects.resize(2);

  collision_objects[0].id = "bin";
  collision_objects[0].header.frame_id = "world";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;

  ros::waitForShutdown();
  return 0;
}
