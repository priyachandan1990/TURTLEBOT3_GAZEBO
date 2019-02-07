/*!
 * @file robomove.h
 *
 * @author Priya Chandanshive <priya.chandanshive@outlook.com>
 *
 * @brief To control the robot based on simulation environment.
 *
 * @date 3 February 2019
 *
 */

#ifndef PROJECT_ROBOMOVE_H
#define PROJECT_ROBOMOVE_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"

#define OBSTACLE_THRESHOLD 0.5

// Free space object
struct FreeSpace{
  float start_position;  /*!< Start position of FreeSpace */
  float space_size;  /*!< size of FreeSpace (angular) */
};


class RoboMove
{

public:
    /*!
     * @brief Constructor for Robot Controller
     *
     * @param target_location_x target location for final robot location (X)
     * @param target_location_y target location for final robot location (Y)
     */
    RoboMove(ros::NodeHandle nh, double target_location_x, double target_location_y);

private:

    /*!
     * @brief Callback for odometry message reports from the simulator
     *
     * @param msg message obtained from Gazebo simulation for robot odometry
     */
    void roboCurrentOdom(const nav_msgs::Odometry::ConstPtr &msg);

    /*!
     *
     * @brief Callback for laserSensor Parser for obstacle detection and decision making.
     *        This function also checks if the robot is closer to an obstacle and turns it
     *        in order to avoid collusion
     *
     *        See below location for message details
     *        http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
     *
     * @param msg message got from the Laser scanner on the turtlebot
     */
    void laserSensorParser(const sensor_msgs::LaserScan::ConstPtr &msg);
    // void LaserBasedFreeSpaceDeterminer(const sensor_msgs::LaserScan::ConstPtr &scan);

    /*!
     * @brief Logic for a non obstacle path propagation of the robot.
     *        Perform rotation adjustment of the robot w.r.t target location.
     *
     * @return Robot commands to perform speed and rotation in simulation
     */
    geometry_msgs::Twist nonObstaclePathLogic(void);

    /*!
     * @brief To determine the direction in which the robot needs to turn in case of obstacle present in its path.
     *         1) When Obstacle present in left side of robot --> Turn Right
     *         2) When Obstacle present in right side of robot --> Turn left
     *         3) When Obstacle present in front (perfect 90 degrees) of robot --> Turn Right  (Default direction)
     *
     * @param min_range_from_onstacle minimum distance from the obstacle (obtained from Laser sensor)
     * @param nearest_angle_to_obstacle angle to the nearest obstacle
     * @return Robot commands to perform speed and rotation in simulation
     */
    geometry_msgs::Twist ObstacleDetectionLogic(float &min_range_from_onstacle, float &nearest_angle_to_obstacle);

    double curr_robo_theta;
    geometry_msgs::Point curr_robo_loc;  /*!< Current turtlebot rotation angle */
    geometry_msgs::Point target_robo;  /*!< Position of the target to which the robot needs to move */

    // ros node
    ros::NodeHandle tgaNode;  /*!< ROS node handle */

    // Subscribers for topics
    ros::Subscriber odomSub_;  /*!< Subscriber handle to Robot Odometry */
    ros::Subscriber laserScannerSub_;  /*!< Subscriber handle to LaserScanner Sensor (LiDAR) */

    // Advertiser to topic
    ros::Publisher roboCommanderPub_;  /*!< Publisher to robot control */

};

#endif //PROJECT_ROBOMOVE_H
