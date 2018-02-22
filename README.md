# barrett_wam_gazebo_sim

## Cloning

Since this repo contains a submodule, `barrett_model`, you must clone using the following command (as described in the [git documentation](https://git-scm.com/book/en/v2/Git-Tools-Submodules))

```
git clone --recurse-submodules https://github.com/saszaz/barrett_wam_gazebo_sim
```
To change to the project-relevant branch for this repo:

```
cd barrett_model
git checkout --track origin/my_devel
```

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

### Trajectory Following Test

Within the `barrett_wam_moveit_control` package, you can generate an elliptical trajectory using the matlab file `scripts/path_generator.m` and making sure to set the appropriate parameters. This will create a `cart_path_ellipse_n_***.csv` under the `trajectories` directory. 

Currently, the .csv file to use for trajectory following can be set in `src/move_group_interface.cpp`. **TO-DO**: read this selection (along with other parameters) from a config file to avoid re-compilation.

You can then run the waypoint-following script through the launch file:

```
roslaunch barrett_wam_moveit_control moveit_control.launch
```
The arm will attempt to move to a pre-set start configuration, and execute the trajectory.


