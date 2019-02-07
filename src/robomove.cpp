/*!
 * @file robomove.cpp
 *
 * @author Priya Chandanshive <priya.chandanshive@outlook.com>
 *
 * @brief To control the robot based on simulation environment.
 *
 * @date 3 February 2019 
 */

#include <math.h>
#include <cstdlib>

#include "robomove.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h>

RoboMove::RoboMove(ros::NodeHandle nh, double target_location_x, double target_location_y) :
        tgaNode(nh),
        curr_robo_theta(0.0)
{
    curr_robo_loc.x = 0.0;
    curr_robo_loc.y = 0.0;
    target_robo.x = target_location_x;
    target_robo.y = target_location_y;

    // Subscribe to the message from Gazebo simulation topics
    odomSub_ = tgaNode.subscribe("odom", 10, &RoboMove::roboCurrentOdom, this);
    laserScannerSub_ = tgaNode.subscribe("scan", 10, &RoboMove::laserSensorParser, this);

    // Advertise to the Gazebo Robo control topic
    roboCommanderPub_ = tgaNode.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
}

void
RoboMove::roboCurrentOdom(const nav_msgs::Odometry::ConstPtr &msg) {

    // std::cout << "robo odom" << std::endl;
    // position
    curr_robo_loc.x = msg->pose.pose.position.x;
    curr_robo_loc.y = msg->pose.pose.position.y;

    std::cout << "Current Loc: ( " << curr_robo_loc.x
              << ", " <<  curr_robo_loc.y << ")"
              << std::endl;

    // Quaternions for rotations
    double x_quarternion = msg->pose.pose.orientation.x;
    double y_quarternion = msg->pose.pose.orientation.y;
    double z_quarternion = msg->pose.pose.orientation.z;
    double w_quarternion = msg->pose.pose.orientation.w;

    // Convert Quarternions to Yaw, Pitch and Roll Angles (Euclidean)
    double roll, pitch, yaw;
    tf::Quaternion qt(x_quarternion, y_quarternion, z_quarternion, w_quarternion);
    tf::Matrix3x3 m(qt);
    m.getRPY(roll, pitch, yaw);
    curr_robo_theta = yaw;  // interested in the yaw angle since its around the z axis of the robot
}

void
RoboMove::laserSensorParser(const sensor_msgs::LaserScan::ConstPtr &msg) {

    // std::cout << "Laser Sensor Parser " << std::endl;

    geometry_msgs::Twist robo_cmd;  // command control for publication

    // take each message from the Laser scanner as a 360 degree laser input
    float min_range_from_onstacle = msg->ranges[0]; // init with first value from the sensor
    float nearest_angle_to_obstacle = 0.0F; // init as 0

    // find out number of cycles
    int total_scan_angles = (scan->angle_max - scan->angle_min);
    total_scan_angles = total_scan_angles/scan->angle_increment

    // per cycle, parse through each range and
    // find the smallest range and corresponding angle to obstacle
    for (int cy = 0; cy < total_scan_angles; ++cy) {
        if (msg->ranges[cy] < min_range_from_onstacle) {
            min_range_from_onstacle = msg->ranges[cy];
            nearest_angle_to_obstacle = cy / 2;
        }
        // std::cout << "Smallest range --> " << min_range_from_onstacle << std::endl;
        //          << " Angle to obstacle --> " << nearest_angle_to_obstacle
        //          << std::endl;
    }

    // depending on the sensor values, make decision for robo movement
    if (min_range_from_onstacle <= 0.3)  // Threshold value for Laser scan
    {
        // if the object on the right side of the robot, turn it to the left side
        // and vice versa
        robo_cmd = ObstacleDetectionLogic(min_range_from_onstacle, nearest_angle_to_obstacle);

    } else  // robot not in vicinity of the obstacle
    {
        robo_cmd = nonObstaclePathLogic();
    }

    // present the robot with the decision for movement (publish)
    roboCommanderPub_.publish(robo_cmd);
}

geometry_msgs::Twist
RoboMove::nonObstaclePathLogic(void) {
    geometry_msgs::Twist robo_cmd;

    // find the angle between current robot position and the target location
    double x_diff = target_robo.x - curr_robo_loc.x;
    double y_diff = target_robo.y - curr_robo_loc.y;
    double angle_to_goal = atan2(y_diff, x_diff);
    double distance_from_goal = sqrt(x_diff * x_diff + y_diff * y_diff);

    std::cout << "Angle to goal location --> " << angle_to_goal << std::endl;
    std::cout << "Distance to goal location --> " << distance_from_goal << std::endl;

    if (std::abs(angle_to_goal - curr_robo_theta) > 0.1) {
        std::cout <<  "Correcting path to point to logic" << std::endl;
        // if the robot is having a deviation of 6 degrees from target, give a slight correction
        robo_cmd.linear.x = 0.2;
        robo_cmd.angular.y = angle_to_goal;  // path correction provided
    } else {
        std::cout <<  "Going Straight" << std::endl;
        robo_cmd.linear.x = 1.0;  // full speed ahead !
        robo_cmd.angular.y = 0.0;  // no corrections needed
    }

    return robo_cmd;
}

geometry_msgs::Twist
RoboMove::ObstacleDetectionLogic(float &min_range_from_onstacle,
                                 float &nearest_angle_to_obstacle) {

    geometry_msgs::Twist robo_cmd;

    if (nearest_angle_to_obstacle < 90.0) {
        robo_cmd.linear.x = 0.1;
        robo_cmd.angular.z = 1.0; // Turn Left
        std::cout << "Turning Left" << std::endl;
    } else if (nearest_angle_to_obstacle > 90.0){
        robo_cmd.linear.x = 0.1;
        robo_cmd.angular.z = -1.0; // Turn Right
        std::cout << "Turning Right" << std::endl;
    }
    else
    {
        robo_cmd.linear.x = -0.1;  // Go reverse
        robo_cmd.angular.z = 0.0; // Turn Right
        std::cout << "Going back" << std::endl;
    }

    return robo_cmd;
}


// Another method to check for obstacle detection --> FreeSpace based
// void RoboMove::LaserBasedFreeSpaceDeterminer(
//                   const sensor_msgs::LaserScan::ConstPtr& scan)
// {
//
//     FreeSpace curr_free_space;
//     float max_free_space_start = 0.0;
//     float max_free_space_finish = 0.0;
//     float average_free_space = 0.0;
//     bool isFreeSpace = false;
//     float max_free_space = 120;  // change this to a better value
//
//     // find the number of angles in which the Sensor has scanned
//     int scan_angles_total = (scan->angle_max - scan->angle_min) / scan->angle_increment;
//
//     // std::cout << "Total number of scan angles = " << scan_angles_total << std::endl;
//
//     // data collector
//     float ranges[scan_angles_total - 1];
//     for (int i=0; i < scan_angles_total; ++i){
//        ranges[i] = scan->ranges[i];
//     }
//
//     ranges[0] = 0.9; // sim demarkation
//     ranges[scan_angles_total-1] = 0.9;
//
//     // Lets give a obstacle THRESHOLD for demarking closeness to an obstacle
//     for (int i=0; i < scan_angles_total; ++i){
//        if ((ranges[i] < OBSTACLE_THRESHOLD) && ranges[i] > 0.5) {
//           ranges[i] = 1;
//        } else {
//           ranges[i] = 0;
//        }
//      }
//
//
//      // lets start a freespace scan on laser data
//      for (int i=0; i < scan_angles_total; ++i){
//
//        // Check for beginning of freespace
//        if ((ranges[i] == 0) && (!isFreeSpace)){
//          isFreeSpace = true;
//          curr_free_space.start_position = i;
//        }
//
//        // If already in freespace gathering mode, find the freespace size
//        if (isFreeSpace){
//          curr_free_space.space_size++; // increment gap counter
//        }
//
//        // check for end of a free space
//        if ((ranges[i] == 1) && (isFreeSpace)){
//          isFreeSpace = false; // not a freespace anymore!!
//
//          if (curr_free_space.space_size > max_free_space){
//            max_free_space_start = curr_free_space.start_position * scan->angle_increment * 180/M_PI;
//            max_free_space = curr_free_space.space_size;
//            max_free_space_finish = max_free_space_start + max_free_space * scan->angle_increment * 180/M_PI;
//            average_free_space = (max_free_space_start + max_free_space_finish)/2;
//          }
//
//          // reset everything !
//          curr_free_space.start_position = 0.0;
//          curr_free_space.space_size = 0.0;
//        }
//      }
//
//      // Create robot commands
//      float ag = 0.01 * (-1) * (90 - average_free_space);
//      std::cout << "Angle of turn --> " << ag << std::endl;
//      geometry_msgs::Twist robo_cmd;
//      robo_cmd.linear.x = 0.3;  //Constant Velocity
//      robo_cmd.angular.z = ag;
//
//      roboCommanderPub_.publish(robo_cmd);
//
// }
