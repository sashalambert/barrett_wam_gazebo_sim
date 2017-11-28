#include <math.h>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// #define HOME
#define CARTESIAN
// #define RANDOM

void readTrajFile(  std::vector<std::vector<float>> * coord, std::string path) 
{

  std::ifstream file(path, std::ios::in);
  std::string line; 

  while( std::getline( file, line ) )
  {
    std::istringstream iss( line );
    std::string result;
    std::vector<float> p;
    while( std::getline( iss, result, ',' ) )
      {
        p.push_back(atof( result.c_str() ));
      }
    // ROS_INFO("%5.3f  %5.3f", p[0], p[1]);
    coord->push_back(p);
  }

  file.close();
}


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
  bool success;


#ifdef HOME
  /*----------------------------------------------------------------------------------------------*/
  // MOVE TO HOME POSITION

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = -M_PI/2;
  joint_group_positions[2] = 0.0;
  joint_group_positions[3] = 0.0; 
  // joint_group_positions[3] = M_PI/2; 
  joint_group_positions[4] = 0.0;
  joint_group_positions[5] = 0.0;
  joint_group_positions[6] = 0.0;

  // move_group.setMaxVelocityScalingFactor(0.1);

  move_group.setJointValueTarget(joint_group_positions);
  success = move_group.plan(my_plan);
  move_group.execute(my_plan);

  ros::Duration(5.0).sleep();

#endif

#ifdef CARTESIAN
  /*----------------------------------------------------------------------------------------------*/
  // FOLLOW CARTESIAN WAYPOINTS

  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();

  /*----------------------------------------------------------------------------------------------*/
  // Start at at pre-set pose

  moveit::core::RobotStatePtr state = move_group.getCurrentState();
  std::vector<double> joint_group_start;
  state->copyJointGroupPositions(joint_model_group, joint_group_start);

  joint_group_start[0] = 0.5656109446;
  joint_group_start[1] = -1.4139894723;
  joint_group_start[2] = 1.8147345939;
  joint_group_start[3] = 1.7912691745;
  joint_group_start[4] = -3.1911434401;
  joint_group_start[5] = 1.1331839406;
  joint_group_start[6] = -0.3702485934;

  move_group.setJointValueTarget(joint_group_start);
  success = move_group.plan(my_plan);
  move_group.execute(my_plan);

  /*----------------------------------------------------------------------------------------------*/

  // Current end-eff pose
  geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose);

  //  Read 2D coordinates of trajectory
  std::string traj_fn = "/home/sasha/catkin_kinetic_ws/src/barrett_wam_gazebo_sim/barrett_wam_moveit_control/scripts/trajectories/cart_path_ellipse_n_100.csv";
  std::vector<std::vector<float>> pos;
  readTrajFile(&pos, traj_fn);

  double scaling_fact=0.2;
  int n_loops = 1;

  for (int l=0; l<n_loops; l++) {
    for (int pt=0; pt<pos.size()-1; pt++){

      // Read as y-z coordinates, current pose as origin.
      geometry_msgs::Pose target_pose = current_pose;
      target_pose.position.y += pos[pt][0]*scaling_fact;
      target_pose.position.z += pos[pt][1]*scaling_fact;

      ROS_INFO("point y: %5.3f, z: %5.3f",pos[pt][0],pos[pt][1]);
      waypoints.push_back(target_pose); 
    }
  }


  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  
  // move_group.setMaxVelocityScalingFactor(0.1);


    // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;

  // No more than jump_threshold is allowed as change in distance in the configuration space of 
  // the robot (this is to prevent 'jumps' in IK solutions)
  const double jump_threshold = 0.0;
  // Step size of at most eef_step meters between end effector configurations of consecutive points.
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