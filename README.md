RotorS
===============

RotorS is a MAV gazebo simulator.
It provides some multirotor models such as the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/), the [AscTec Pelican](http://www.asctec.de/en/uav-uas-drone-products/asctec-pelican/), or the [AscTec Firefly](http://www.asctec.de/en/uav-uas-drone-products/asctec-firefly/), but the simulator is not limited for the use with these multicopters.

There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the [VI-Sensor](http://wiki.ros.org/vi_sensor), which can be mounted on the multirotor.

This package also contains some example controllers, basic worlds, a joystick interface, and example launch files.

Below we provide the instructions necessary for getting started. See RotorS' wiki for more instructions and examples (https://github.com/ethz-asl/rotors_simulator/wiki).

If you are using this simulator within the research for your publication, please cite:
```bibtex
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```
Installation Instructions
-------------------------

 1. Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-octomap-ros python-wstool python-catkin-tools
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/indigo/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 ```
 > **Note** for setups with multiple workspaces please refer to the official documentation at http://docs.ros.org/independent/api/rosinstall/html/ by replacing `rosws` by `wstool`.
 3. Get the simulator and additional dependencies

 ```
 $ cd ~/catkin_ws/src
 $ git clone git@github.com:ethz-asl/rotors_simulator.git
 $ git clone git@github.com:ethz-asl/mav_comm.git
 ```
  > **Note** if you want to use `wstool` you can replace the above commands with
    ```
    wstool set --git local_repo_name git@github.com:organization/repo_name.git
    ```
  > **Note** if you want to build and use the `gazebo_mavlink_interface` plugin you have to get MAVROS as an additional dependency from link below. Follow the installation instructions provided there and build all of its packages prior to building the rest of your workspace. 
    ```
    https://github.com/mavlink/mavros
    ```
 4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin init  # If you haven't done this before.
   $ catkin build
   ```

 5. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

Basic Usage
-----------

Launch the simulator with a hex-rotor helicopter model, in our case, the AscTec Firefly in a basic world.

```
$ roslaunch rotors_gazebo mav_empty_world.launch mav_name:=firefly world_name:=basic
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database.

The simulator starts by default in paused mode. To start it you can either
 - use the Gazebo GUI and press the play button
 - or you can send the following service call.

   ```
   $ rosservice call gazebo/unpause_physics
   ```

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/launch`.

The `world_name` argument looks for a .world file with a corresponding name in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds`. By default, all launch files, with the exception of those that have the world name explicitly included in the file name, use the empty world described in `basic.world`.

### Getting the multicopter to fly

To let the multicopter fly you need to generate thrust with the rotors, this is achieved by sending commands to the multicopter, which make the rotors spin.
There are currently a few ways to send commands to the multicopter, we will show one of them here.
The rest is documented [here](../../wiki) in our Wiki.
We will here also show how to write a stabilizing controller and how you can control the multicopter with a joystick.

#### Send direct motor commands

We will for now just send some constant motor velocities to the multicopter.

```
$ rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [100, 100, 100, 100, 100, 100]}'
```

> **Note** The size of the `motor_speed` array should be equal to the number of motors you have in your model of choice (e.g. 6 in the Firefly model).

You should see (if you unpaused the simulator and you have a multicopter in it), that the rotors start spinning. The thrust generated by these motor velocities is not enough though to let the multicopter take off.
> You can play with the numbers and will realize that the Firefly will take off with motor speeds of about 545 on each rotor. The multicopter is unstable though, since there is no controller running, if you just set the motor speeds.


#### Let the helicopter hover with ground truth odometry

You can let the helicopter hover with ground truth odometry (perfect state estimation), by launching:

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

#### Create an attitude controller

**TODO(ff):** `Write something here.`

#### Usage with a joystick

**TODO(ff):** `Write something here.`

#### Using a joystick to control multiple multicopters

You can use a joystick to control a single multicopter by launching:

```
$ roslaunch rotors_gazebo mav_with_waypoint_publisher_joy.launch
```

Wait for a few seconds for the simulation to load. When the multicopter start hovering a little bit above the ground, you can then use the left analog stick to control the multicopter's position on the XY plane. Pushing the right analog stick forward or backward will control the elavation upward or downward respectively.

You can also use the same launch file to spawn multiple multicopters, by launching with an extra `no_spawn` argument:

```
$ roslaunch rotors_gazebo mav_with_waypoint_publisher_joy.launch no_spawn:=true
```

This will launch an empty gazebo scene in which you can spawn a multicopter with the command:

```
$ roslaunch rotors_gazebo mav_with_waypoint_publisher_joy.launch no_world:=true mav_name:=firefly robot_name:=robot1
```

Upon exiting, the launch file does not remove the spawned model from the scene. However, the launch file does remove any existing model with the same name as the `robot_name` parameter when starting up. All multicopters spawned with this method accept the same input commands from the joystick.

The following is the list of parameters for the launch file:
  * `robot_name` *(string, default=`$(arg mav_name)`)* name of the multicopter's model in Gazebo  
  * `no_world`   *(boolean, default=false)* do not start Gazebo
  * `no_spawn`   *(boolean, default=false)* do not spawn a multicopter
  * `gdb`        *(boolean, default=false)* debug the position controller and the joy to waypoint interface with gdb
  * `plot`       *(boolean, default=true)* display position and orientation plots
  * `x`          *(double, default=0.0)* the starting X position
  * `y`          *(double, default=0.1)* the starting Y position
  * `z`          *(double, default=0.1)* the starting Z position

An example launch file for spawning four multicopters of the same model at once is also provided.

```
$ roslaunch rotors_gazebo multicopters_joy.launch mav_name:=firefly group_name:=group1
```

