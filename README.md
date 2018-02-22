# barrett_wam_gazebo_sim

## Running the code

### Launch the gazebo interface 

The `barrett_wam.launch` file will start the Gazebo simulator, load the urdf model, and start the controller interfaces. The `ctrl_mode` argument can be set to `position` (default), `velocity`, or `effort`.

```
roslaunch barrett_wam_gazebo barrett_wam.launch ctrl_mode:=position
```
