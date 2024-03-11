/*
 * Code modified by Xiaoling Zeng
 * for CSCE 574 Homework 1
 * Spring 2024
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

class RandomWalk {
  public:
	// Construst a new RandomWalk object and hook up this ROS node
	// to the simulated robot's velocity control and laser topics
	RandomWalk(ros::NodeHandle& nh):
 	fsm(FSM_MOVE_FORWARD),
	rotateStartTime(ros::Time::now()),
	rotateDuration(0.f) {
	// Initialize random time generator
		srand(time(NULL));

		// Advertise a new publisher for the simulated robot's velocity command topic
		// (the second argument indicates that if multiple command messages are in
		// the queue to be sent, only the last command will be sent)
		
		commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		poseSub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &RandomWalk::poseCallback, this);
		pathPub = nh.advertise<nav_msgs::Path>("robot_trajectory", 10);
		trajectory.header.frame_id = "odom";
		// Subscribe to the simulated robot's laser scan topic and tell ROS to call
		// this->commandCallback() whenever a new message is published on that topic
		laserSub = nh.subscribe("/scan", 1, &RandomWalk::commandCallback, this);
	};

	// Send a velocity command
	void move(double linearVelMPS, double angularVelRadPS) {
		geometry_msgs::Twist msg; // The default constructor will set all commands to 0
		msg.linear.x = linearVelMPS;
		msg.angular.z = angularVelRadPS;
		commandPub.publish(msg);
	};

	void poseCallback(const nav_msgs::Odometry::ConstPtr& Msg)
    {
        RobotPose.pose = Msg->pose.pose;
    }

	void publishTrajectory(){
        trajectory.poses.push_back(RobotPose);
        pathPub.publish(trajectory);
    	};

	// Process the incoming laser scan message
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		if (fsm == FSM_MOVE_FORWARD) {
			// Compute the average range value between MIN_SCAN_ANGLE and MAX_SCAN_ANGLE
			//
			// NOTE: ideally, the following loop should have additional checks to ensure
			// 	that indices are not out of bounds, by computing:
			//
			//   -currAngle = msg->angle_min + msg->angle_increment*currIndex
			//
			//       and then ensuring that currAngle <= msg->angle_max
			unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
			unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
			float closestRange = msg->ranges[minIndex];
			float currAngle;
			for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
			currAngle = msg->angle_min + msg->angle_increment*currIndex;
				if (msg->ranges[currIndex] <closestRange) {
                    			if (currAngle <= msg->angle_max)
                    				{closestRange = msg->ranges[currIndex];}
				}
			}
			ROS_INFO_STREAM("Range: " << closestRange);
			

			// TODO: if range is smaller than PROXIMITY_RANGE_M, update fsm and rotateStartTime,  
      			// and also choose a reasonable rotateDuration (keeping in mind of the value of ROTATE_SPEED_RADPS) 
			// 
			// HINT: you can obtain the current time by calling: 
			// 
			// - ros::Time::now() 
			// 
			// HINT: you can set a ros::Duration by calling: 
			// 
			// - ros::Duration(DURATION_IN_SECONDS_FLOATING_POINT)  // 
			// HINT: you can generate a random number between 0 and 99 by calling:  // 
			// - rand() % 100 
			// 
			// see http://www.cplusplus.com/reference/clibrary/cstdlib/rand/ for more details  
      			/////////////////////// ANSWER CODE BEGIN /////////////////// 
            		if (closestRange < PROXIMITY_RANGE_M)
                    	{
                        fsm = FSM_ROTATE;
                        ROS_INFO("Make a turn!");
                        rotateStartTime = ros::Time::now();
                        rotateDuration = ros::Duration(rand() % 100);
                        ROS_INFO_STREAM("rotateDuration: " << rotateDuration);
                    	}	
            		else	
            		{move(FORWARD_SPEED_MPS, 0);}	
			/////////////////////// ANSWER CODE END /////////////////// 
		}
	};
	
	// Main FSM loop for ensuring that ROS messages are
	// processed in a timely manner, and also for sending
	// velocity controls to the simulated robot based on the FSM state
	void spin() {
		ros::Rate rate(10); // Specify the FSM loop rate in Hz
		while(ros::ok()) { // Keep spinning loop until user presses Ctrl+C	
			// TODO: Either call:
			// - move(0, ROTATE_SPEED_RADPS); // Rotate right 
			// or 
			// - move(FORWARD_SPEED_MPS, 0); // Move foward 
			// depending on the FSM state; also change the FSM state when appropriate  
      			/////////////////////// ANSWER CODE BEGIN /////////////////// 
          
            		if (fsm == FSM_MOVE_FORWARD)
            			{move(FORWARD_SPEED_MPS, 0);}
            		else if (ros::Time::now() > rotateStartTime + rotateDuration)
            			{move(FORWARD_SPEED_MPS, 0);
            			fsm = FSM_MOVE_FORWARD;}
            		else
            			{move(0, ROTATE_SPEED_RADPS);}
			publishTrajectory();
			/////////////////////// ANSWER CODE END /////////////////// 
			
			ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
			
			rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
		}
	};

	enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};

	// Tunable parameters
  // TODO: tune parameters as you see fit 
	constexpr static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
	constexpr static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
	constexpr static float PROXIMITY_RANGE_M = 0.8; // Should be smaller than  sensor_msgs::LaserScan::range_max 
	constexpr static double FORWARD_SPEED_MPS = 1.0;
	constexpr static double ROTATE_SPEED_RADPS = M_PI/4;

  	protected:
	ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
	ros::Publisher pathPub;
    ros::Subscriber poseSub;

    nav_msgs::Path trajectory;
    geometry_msgs::PoseStamped RobotPose;

	enum FSM fsm; // Finite state machine for the random walk algorithm
	ros::Time rotateStartTime; // Start time of the rotation
	ros::Duration rotateDuration; // Duration of the rotation
};

int main(int argc, char**argv) {
	ros::init(argc, argv, "random_walk"); // Initiate new ROS node named "random_walk"
	ros::NodeHandle n;
	RandomWalk walker(n); // Create new random walk object
	walker.spin(); // Execute FSM loop
	return 0;
};
