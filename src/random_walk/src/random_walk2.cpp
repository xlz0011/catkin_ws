#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

class RandomWalk {
public:
    RandomWalk(ros::NodeHandle& nh) {
        
        commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		poseSub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &RandomWalk::poseCallback, this);
		laserSub = nh.subscribe("base_scan", 1, &RandomWalk::commandCallback, this);

    }

    void translate(double distance) {
        double velocity = 0.1;
        double duration = distance / velocity;
        geometry_msgs::Twist msg;
        msg.linear.x = velocity;
        commandPub.publish(msg);
        ros::Duration(duration).sleep();
        msg.linear.x = 0;
        commandPub.publish(msg);
    }

    void rotateRel(double angle) {
        double angular_velocity = M_PI / 6; 
        double duration = std::abs(angle) / angular_velocity;
        geometry_msgs::Twist msg;
        msg.angular.z = angle > 0 ? angular_velocity : -angular_velocity;
        commandPub.publish(msg);
        ros::Duration(duration).sleep();
        msg.angular.z = 0;
        commandPub.publish(msg);
    }

    void runTests() {
        translate(1);
        rotateRel(M_PI / 6); 
        rotateRel(-M_PI / 6); 
    }



private:

    ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
    ros::Subscriber poseSub;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "random_walk");
    RandomWalk random_walk(n);
    RandomWalk.runTests();
    ros::spin();
    return 0;
}
