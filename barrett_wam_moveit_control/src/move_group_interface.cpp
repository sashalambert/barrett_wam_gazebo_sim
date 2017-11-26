#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

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

  bool success = move_group.plan(my_plan);
  move_group.execute(my_plan);

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

}