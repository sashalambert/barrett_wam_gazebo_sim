/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>


#include <moveit/robot_model_loader/robot_model_loader.h>

// Executing plans
#include <moveit/move_group_interface/move_group_interface.h>

// Planning Pipeline (Traj. Parameterization)
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <math.h>
#include <fstream>

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");


  // Connect to planning groups for execution
  moveit::planning_interface::MoveGroupInterface move_group("wam_arm");
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup("wam_arm");

  // RobotModelLoader
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  // PlanningScene maintains state of the world
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // Planning Pipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node_handle, "/wam/move_group/planning_plugin", "/wam/move_group/request_adapters"));


  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(1.0);
  sleep_time.sleep();


  /*----------------------------------------------------------------------------------------------*/
  // Start at at pre-set pose


  /* Get the joint model group */
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  robot_state.setJointGroupPositions(joint_model_group,joint_group_positions);

  // Set the joint space goal
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> joint_group_values(7, 0.0);
  joint_group_values[0] = 0.5656109446;
  joint_group_values[1] = -1.4139894723;
  joint_group_values[2] = 1.8147345939;
  joint_group_values[3] = 1.7912691745;
  joint_group_values[4] = -3.1911434401;
  joint_group_values[5] = 1.1331839406;
  joint_group_values[6] = -0.3702485934;
  goal_state.setJointGroupPositions(joint_model_group, joint_group_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  req.group_name = "wam_arm";
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // Call the pipeline and visualize the trajectory
  planning_pipeline->generatePlan(planning_scene, req, res);
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/wam/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;


  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = response.trajectory;

  // trajectory of type robot_trajectory::RobotTrajectoryPtr
  // std::cout <<"VELOCITY: " << res.trajectory_.points() << std::endl;
  // std::cout <<"VELOCITY: " << res.trajectory_.getWaypoint(1).getVariableVelocities() << std::endl;

  // ROS_INFO("Executing:");
  // move_group.execute(plan);

  sleep_time.sleep();

  /*----------------------------------------------------------------------------------------------*/

  // STILL USES PLANNER HERE


  ROS_INFO("Starting Waypoint Trajectory");

  // Current end-eff pose
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  std::vector<geometry_msgs::PoseStamped> waypoints;
  waypoints.push_back(current_pose);

  //  Read 2D coordinates of trajectory
  std::string traj_fn = "/home/sasha/catkin_kinetic_ws/src/barrett_wam_gazebo_sim/barrett_wam_moveit_control/scripts/trajectories/cart_path_ellipse_n_100.csv";
  // std::string traj_fn = "/home/sasha/catkin_kinetic_ws/src/barrett_wam_gazebo_sim/barrett_wam_moveit_control/scripts/trajectories/cart_path_ellipse_n_200.csv";
  // std::string traj_fn = "/home/sasha/catkin_kinetic_ws/src/barrett_wam_gazebo_sim/barrett_wam_moveit_control/scripts/trajectories/cart_path_ellipse_n_25.csv";
  std::vector<std::vector<float>> pos;
  readTrajFile(&pos, traj_fn);

  double scaling_fact=0.4;
  int n_loops = 3;
  std::vector<double> offset = {0.1,0.0,0.4};

  current_pose.pose.position.x += offset[0];
  current_pose.pose.position.y += offset[1];
  current_pose.pose.position.z += offset[2];


  for (int l=0; l<n_loops; l++) {
    for (int pt=0; pt<pos.size()-1; pt++){

      // Read as y-z coordinates, current pose as origin.
      geometry_msgs::PoseStamped target_pose = current_pose;
      target_pose.pose.position.y += pos[pt][0]*scaling_fact;
      target_pose.pose.position.z += pos[pt][1]*scaling_fact;

      // ROS_INFO("point y: %5.3f, z: %5.3f",pos[pt][0],pos[pt][1]);
      waypoints.push_back(target_pose); 

    }
  }

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // req.path_constraints.clear();
  moveit_msgs::TrajectoryConstraints path_points;
  for (int wp=0; wp<waypoints.size()-1; wp++){

      moveit_msgs::Constraints path_point = 
        kinematic_constraints::constructGoalConstraints(move_group.getEndEffectorLink(), waypoints[wp], tolerance_pose, tolerance_angle);
      path_points.constraints.push_back(path_point);
  }
  req.trajectory_constraints = path_points;
  moveit_msgs::Constraints path_end =
      kinematic_constraints::constructGoalConstraints(move_group.getEndEffectorLink(), waypoints.back(), tolerance_pose, tolerance_angle);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(path_end);


  /* Set velocity & accel. scaling */
  req.max_velocity_scaling_factor = 0.01;
  req.max_acceleration_scaling_factor = 0.01;

  // Call the pipeline and visualize the trajectory
  planning_pipeline->generatePlan(planning_scene, req, res);
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);
  plan.trajectory_ = response.trajectory;


  move_group.execute(plan);

  return 0;
}
