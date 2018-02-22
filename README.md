# barrett_wam_gazebo_sim

## Running the code

### Launch the gazebo interface 

The `barrett_wam.launch` file will start the Gazebo simulator, load the urdf model, and start the controller interfaces. The `ctrl_mode` argument can be set to `position` (default), `velocity`, or `effort`. The robot state, including joint values and tf transforms are published on corresponding rostopics.

```
roslaunch barrett_wam_gazebo barrett_wam.launch ctrl_mode:=position
```

### Launch the MoveIt! interface

This will start the `move_group` planning interface, and allow path planning using the MoveIt! library.

```
roslaunch barrett_wam_gazebo barrett_wam_moveit_planning_execution.launch debug:=true sim:=true
```

### Launch RViz (Optional)

This includes a manual user-interface for setting start/goal positions of the robot end-effector, and generating plans from selected OMPL path planners. 

```
roslaunch barrett_wam_moveit_config moveit_rviz.launch config:=true
```
