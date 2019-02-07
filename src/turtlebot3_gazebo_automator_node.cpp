/*!
 * @file turtlebot3_gazebo_automator_node.cpp
 *
 * @author Priya Chandanshive <priya.chandanshive@outlook.com>
 *
 * @brief ROS node for controlling robot through a Gazebo based simulation from
 *        one position to another (avoiding obstacles on the way)
 *
 * @date 3 February 2019
 *
 */

#include <iostream>

// ROS main
#include "ros/ros.h"

// internal node includes
#include "robomove.h"

/*!
 * @brief Application Entry
 */
int main(int argc, char **argv)
{

    std::cout << "Starting the node for automator" << std::endl;

    // init the ros module for handles
    ros::init(argc, argv, "turtlebot_gazebo_automator");

    // create a node for this plugin
    ros::NodeHandle nh;

    // start node handle for robo control
    RoboMove roboController(nh, 2.0, 2.0);

    // ros runtime activate
    ros::spin();

    return 0;
}
