#include <math.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#define CARTESIAN
// #define RANDOM

int main(int argc, char **argv)
{
  ros::init(argc, argv, "barrett_wam_moveit_interface");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "wam_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Get plannning group
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("move_group_interface", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("move_group_interface", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  /*----------------------------------------------------------------------------------------------*/
  // MOVE TO HOME POSITION

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = -M_PI/2;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = M_PI/2; 
  joint_group_positions[4] = 0.0;
  joint_group_positions[5] = 0.0;
  joint_group_positions[6] = 0.0;

  // move_group.setMaxVelocityScalingFactor(0.1);

  move_group.setJointValueTarget(joint_group_positions);
  bool success = move_group.plan(my_plan);
  move_group.execute(my_plan);

  // ros::Duration(5.0).sleep();

#ifdef CARTESIAN
  /*----------------------------------------------------------------------------------------------*/
  // FOLLOW CARTESIAN WAYPOINTS

  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();

  // Current end-eff pose
  geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose);

  double dist=0.1;

  geometry_msgs::Pose target_pose = current_pose;
  target_pose.position.x += dist;
  waypoints.push_back(target_pose);  

  target_pose.position.x -= dist;
  target_pose.position.y += dist;
  waypoints.push_back(target_pose); 

  target_pose.position.y -= dist;
  target_pose.position.x -= dist;
  waypoints.push_back(target_pose); 

  target_pose.position.x += dist;
  target_pose.position.y -= dist;
  waypoints.push_back(target_pose); 

  target_pose.position.y += dist;
  waypoints.push_back(target_pose); 


  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);


    // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);


  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();

  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;
  move_group.execute(cartesian_plan);
  ros::Duration(3.0).sleep();

#endif

#ifdef RANDOM

  /*----------------------------------------------------------------------------------------------*/
  // GENERATE RANDOM START/GOAL PLANS

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // bool success = move_group.plan(my_plan);
  // move_group.execute(my_plan);

  int iter=0;
  double count=0;

  for (int i=0; i<500; i++ ) {

    ros::Time start = ros::Time::now();

    moveit::core::RobotStatePtr target_state = move_group.getCurrentState();

    move_group.setRandomTarget();

    bool success = move_group.plan(my_plan);
    move_group.execute(my_plan);
    ROS_INFO("Plan %d finished", i);

    ros::Time end = ros::Time::now();

    if (success) {
      iter++;
      count += end.toSec() - start.toSec();
    }

  } 

  double stat= count / (double) iter;

  ROS_INFO("Avg. Exec. time:  %10.6f sec", stat);

#endif	

}