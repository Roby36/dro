# Description of main source files

The following files are the main source files of the project. They are located in the `src` directory.
The source files are written in C++ and divided into submodules. The submodules are as follows:

- `handlers`: contains the source files which handle the abstracted subscribers and pulishers
- `nav`: contains the source files which handle the navigation of the drone. This is where we find the PID controller and the bug3 pathfinding algorithm.
- `transforms`: contains the source files which handle the static and dynamic transforms between the different reference frames.


Before being compiled and executed, the programs below first require:
1. Starting a SITL copter session (from the appropriate directory)\
    `sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console`
2. Starting Gazebo\
    `gazebo --verbose worlds/iris_arducopter_runway.world`
3. Launching mavros (from appropriate directory) by running\
    `roslaunch apm.launch`


## blBroadcaster

This module establishes a node handling the broadcasting of the main transforms required by the program. 

### Topics subscribed
    /mavros/local_position/odom

### Transforms broadcasted
    odom -> base_link
    odom -> base_link_orientation

The *frame_id* `base_link_orientation` has the same rotation as `base_link` and the same origin as `odom`. This frame allows `JController` to convert velocity vectors to the `odom` reference frame (required by `/mavros/setpoint_velocity/cmd_vel`) by direct transformation. 


## JController

This module uses the *ncurses* library to reproduce a joystick controller for the drone. 

### Topics published
    /mavros/setpoint_velocity/cmd_vel

### Services called
    /mavros/set_mode
    /mavros/cmd/arming
    /mavros/cmd/takeoff
    /mavros/cmd/land

Once initialized, the module handles the arming and takeoff of the drone. After takeoff, the drone can be controlled by pressing the appropriate keys set in `enum Keypress`. The drone lands on its current position when `q` is pressed.
The module uses a `tf::TransformListener`, looking up the transform between `odom` and `base_link_orientation`, to transform velocity commands (given in the `base_link` reference frame) to the `odom` frame.

`JController.launch` initiates the `blBroadcaster` and `JController` nodes respectively for an easier execution of the program.



