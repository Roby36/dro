# Description of main source files

The following files are the main source files of the project. They are located in the `src` directory.
The source files are written in C++ and divided into submodules. The submodules are as follows:

- `handlers`: contains the source files which handle the abstracted subscribers and pulishers
- `nav`: contains the source files which handle the navigation of the drone. This is where we find the PID controller and the bug3 pathfinding algorithm.
- `transforms`: contains the source files which handle the static and dynamic transforms between the different reference frames.


## Building
- Install <a href=https://ardupilot.org/dev/docs/building-setup-linux.html>Ardupilot</a> following the instructions on the website. We cloned it into a directory called `~/ardupilot-ws/src`.
- Install <a href=https://ardupilot.org/dev/docs/ros-install.html#installing-mavros>Mavros</a> following the instructions on the website. *Note: the instructions are for Ubuntu Noetic, but we use Melodic*
- Clone this reposiory into `~/catkin_ws/ardupilot-ws/src`
- Run `catkin_make` in the `~/catkin_ws/ardupilot-ws/src/dro` directory
    - Install dependencies as needed
- In `~/.bashrc`, add the following PATH variables:
    ```bash
    PATH=$PATH:$HOME/catkin_ws/ardupilot_ws/src/ardupilot/Tools/autotest
    PATH=/usr/lib/ccache:$PATH
    PATH="$PATH:$HOME/.local/bin"
    PATH="$PATH:$HOME/.local/bin"
    GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
    GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
    source /usr/share/gazebo/setup.sh
    GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models
    GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
    ```
    - This allows you to run `sim_vehicle.py` from anywhere
- Run `pip install --upgrade pymavlink MAVProxy --user` to install the latest versions of pymavlink and MAVProxy. Run the command again with `pip3`
- Run `sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh` to install the geographiclib datasets for mavros
- Here is the configuration used for Mavros/Ardupilot, which is output to the console upon running the command in step 2 below (see `Running`):

```{yaml}
root@ef76806eeb25:~/catkin_ws/ardupilot_ws/src/ardupilot# cd ~/catkin_ws/ardupilot_ws/src/ardupilot && sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
SIM_VEHICLE: Start
SIM_VEHICLE: Killing tasks
SIM_VEHICLE: Starting up at SITL location
SIM_VEHICLE: WAF build
SIM_VEHICLE: Configure waf
SIM_VEHICLE: "/root/catkin_ws/ardupilot_ws/src/ardupilot/modules/waf/waf-light" "configure" "--board" "sitl"
Setting top to                           : /root/catkin_ws/ardupilot_ws/src/ardupilot 
Setting out to                           : /root/catkin_ws/ardupilot_ws/src/ardupilot/build 
Autoconfiguration                        : enabled 
Checking for program 'python'            : /usr/bin/python3 
Checking for python version >= 3.6.9     : 3.6.9 
Setting board to                         : sitl 
Using toolchain                          : native 
Checking for 'g++' (C++ compiler)        : /usr/bin/g++ 
Checking for 'gcc' (C compiler)          : /usr/bin/gcc 
Checking for c flags '-MMD'              : yes 
Checking for cxx flags '-MMD'            : yes 
CXX Compiler                             : g++ 7.5.0 
Checking for need to link with librt     : not necessary 
Checking for feenableexcept              : yes 
Enabled OpenDroneID                      : no 
Enabled firmware ID checking             : no 
GPS Debug Logging                        : no 
Enabled custom controller                : yes 
Checking for HAVE_CMATH_ISFINITE         : yes 
Checking for HAVE_CMATH_ISINF            : yes 
Checking for HAVE_CMATH_ISNAN            : yes 
Checking for NEED_CMATH_ISFINITE_STD_NAMESPACE : yes 
Checking for NEED_CMATH_ISINF_STD_NAMESPACE    : yes 
Checking for NEED_CMATH_ISNAN_STD_NAMESPACE    : yes 
Checking for header endian.h                   : yes 
Checking for header byteswap.h                 : yes 
Checking for HAVE_MEMRCHR                      : yes 
Configured VSCode Intellisense:                : no 
DC_DSDL compiler                               : /root/catkin_ws/ardupilot_ws/src/ardupilot/modules/DroneCAN/dronecan_dsdlc/dronecan_dsdlc.py 
Source is git repository                       : yes 
Update submodules                              : yes 
Checking for program 'git'                     : /usr/bin/git 
Checking for program 'size'                    : /usr/bin/size 
Benchmarks                                     : disabled 
Unit tests                                     : enabled 
Scripting                                      : enabled 
Scripting runtime checks                       : enabled 
Debug build                                    : disabled 
Coverage build                                 : disabled 
Force 32-bit build                             : disabled 
Checking for program 'rsync'                   : not found 
'configure' finished successfully (1.398s)
```

## Running

Before running the ROS nodes, execute the following commands:
1. Source the ROS workspace and export Python3
    ```bash
    cd ~/catkin_ws/ardupilot_ws/src/dro
    source devel/setup.bash && export PYTHON=/usr/bin/python3
    ```
2. Start an SITL copter session (from the appropriate directory)
    ```bash
    cd ~/catkin_ws/ardupilot_ws/src/ardupilot && sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console -D -G
    ```
3. In a new terminal, start Gazebo with laser scan:
    ```bash
    cd ~/catkin_ws/ardupilot_ws/src/ardupilot_gazebo/build && make install && source /usr/share/gazebo/setup.sh && roslaunch gazebo_ros empty_world.launch world_name:=worlds/iris_arducopter_runway.world verbose:=true
    ```
4. Launch mavros by running:
    ```bash
    cd ~/catkin_ws/ardupilot_ws/src/ardupilot && roslaunch mavros apm.launch
    ```
5. Launch JController.launch by running:
    ```bash
    cd ~/catkin_ws/ardupilot_ws/src/dro/launch && roslaunch JController.launch
    ```

You can now use the following commands to control the drone (defined in JController.h):
- up = 'e'
- down = 'x'
- cw = 'd'
- ccw = 's'
- forwards = 'i'
- backwards = 'm'
- right = 'k'
- left = 'j'
- navtest = 'n'
- bug2test = 'b'
- pidtest = 'p'
- rotate = 'r'
- zntest = 'z'
- twisttest = 'w'
- quit = 'q'



## Known issues and limitations
- Gazebo is very finicky, and sometimes must be restarted. If the drone fails to launch, a reset might be in order.
    - Resetting the world alone is not sufficient. Sometimes Gazebo fails to start entirely.
- The PD controller could be tuned even more to improve the stability and speed of the drone.
- When saving a world in Gazebo, the size of the objects added via the GUI are not saved.
- The PID controller currently functions the same as the differential drive robot, first rotating to have the `base_link` positive x in the desired direction, and then moving in that direction. This is not ideal, as the drone has additional degrees of freedome, and the PID controller should be able to directly adjust the velocity in the desired direction. This is more complex, but could help mitigate overshooting and oscillations.
- Logging could be improved, as we are currently using ROS_INFO to print to the console. This is not ideal, as we are logging directly to STDOUT and not to a file. This could be improved by using ROS logging, or by using a logging library such as spdlog.
- JController module should be extended to allow for continuous testing without restarting Gazebo.
- Unreliable lidar readings at angles (-0.2, 0.2), detecting obstacles
- **Current implementation is Bug2, Bug3 to be added soon.**



## Modules


### Transforms

#### Static Transforms:
- `map` -> `odom`
- `base_link` -> `base_scan` 

#### Dynamic Transforms:
- `odom` -> `base_link`
- `odom` -> `base_link orientation`


#### Topics Subscribed
- /mavros/local_position/odom


The *frame_id* `base_link_orientation` has the same rotation as `base_link` and the same origin as `odom`. This frame allows `JController` to convert velocity vectors to the `odom` reference frame (required by `/mavros/setpoint_velocity/cmd_vel`) by direct transformation. 

### Nav
See documentation in the header files for detailed implementation and design decisions.

#### PID
- Currently uses proportional and derivative terms in two degrees of freedom (similar to differential drive model)

#### VelController
- Holds main navigation algorithms. Currently only Bug2 is implemented, but the module is designed to be easily extended to Bug3, which we are currently pursuing. Uses PID controller to navigate to a point specified by the `JController`.

##### Publishers
- `/mavros/setpoint_velocity/cmd_vel`

##### Subscribers
- `/mavros/sensor/laser_scan`


### Testing

#### JController
- The client-side controller, either used as a joystick (hence "J"Controller) or to launch the Bug algorithms to navigate to a point specified in the script.

##### Topics published
    /mavros/setpoint_velocity/cmd_vel

##### Services called
    /mavros/set_mode
    /mavros/cmd/arming
    /mavros/cmd/takeoff
    /mavros/cmd/land

Once initialized, the module handles the arming and takeoff of the drone. After takeoff, the drone can be controlled by pressing the appropriate keys set in `enum Keypress`. The drone lands on its current position when `q` is pressed. The navigation program is launched when `T` is pressed, and navigates to the point specified in the `navtest` method. The drone lands on the point when it reaches it.

The module uses a `tf::TransformListener`, looking up the transform between `odom` and `base_link_orientation`, to transform velocity commands (given in the `base_link` reference frame) to the `odom` frame.

`JController.launch` initiates the `TransformNode` and `JController` nodes respectively for an easier execution of the program.



