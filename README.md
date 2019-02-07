
GAZEBO ROBOT AUTOMATOR ROS package
===================================

This package contains the codes for a ROBOT propagation through a Gazebo Sim
environment.

## Algorithm

The Node performs the following tasks:
1. Obstacle Detection and avoidance
2. Steering of Robot to a predefined target location

The below sub-sections will give the user a GIST of the workings of these tasks.

### Obstacle Detection and avoidance Logic

The Robot used (turtlebot3) provides the user with `/scan` subscription
topic which provides real time updates from the mounted LaserScan sensor
(Hokuyo). The topic contains the below data:

```
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behaviour (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

Header header            # timestamp in the header is the acquisition time of
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis

float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.
```

Using the `angle_min`, `angle_max` and `angle_increment` values, the number of
Laser readings taken per cycle can be determined using the below logic

```
 num_of_readings =  (angle_max - angle_min)/angle_increment
```

The `ranges` values is an array of size `num_of_readings` which contains
different readings obtained from scans of each angle.

Using these readings, we can find the least range and angle from which the least
range is obtained. These values are then compared against a THRESHOLD radius
(for avoiding collusion). If the value is less than THRESHOLD value, then the
obstacle avoidance logic is activated.

If the angle of least range value is less than 90 degrees, then the obstacle is
located more towards the left side and the robot needs to turned towards right
side for collusion avoidance. Similarly, if the angle of least range value is
more than 90 degrees, then the obstacle is located more towards the right side
and the robot is steered to left side. If the obstacle is straight in front of
the Robot, we try to avoid by defaulting the turn towards right.

### Steering of Robot to a predefined target location

When there is no obstacles on the way of the robot, the angle of current
movement of Robot is calculated using the Target location coordinates and the
current location and rotation of the robot.

The current location and rotation of the Robot is obtained by subscribing the
Node to the `/odom` topic obtained from Gazebo Simulation.

The `Odom` topic gives us the odometry of the Robot. This is basically a message
structure containing the below details:

```
geometry_msgs/PoseWithCovariance pose  # for position & rotation of robot
geometry_msgs/TwistWithCovariance twist
```

## Building this Package

### Prerequisites

Please Check if the following Prerequisites are installed on your machine:

1. Gazebo Simulator (Preferably version 9)
2. ROS Packages ( with supporting Gazebo plugins) - Melodic variant
3. Ubuntu 18.04

### Performing Build

Please install the `turtlebot3_simulations` packages in your machine for Gazebo
Simulations workspace of a turtlebot3 robot:

```
$ cd catkin_ws/src
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd .. && catkin_make install
```

Now copy/clone this code to `catkin_ws/src` folder and perform another
'catkin_make'

```
$ cd catkin_ws/src
$ git clone https://github.com/priyachandan1990/TURTLEBOT3_GAZEBO.git
$ cd .. && catkin_make
$ . ./devel/setup.bash
```

## Running the simulation

1. Start the Gazebo simulation of a turtlebot3 in a Gazebo environment using
   `roslaunch`

   ```
   $ roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```

2. Wait for the Gazebo simulation to come up and show the environment.

3. On another window run the current package as a node using `rosrun`
  ```
  $ rosrun turtlebot3_gazebo_automator turtlebot_gazebo_automator
  ```

4. Observe the robot interaction on simulation window.
