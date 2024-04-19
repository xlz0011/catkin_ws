/*
 * Code modified by YOURNAME
 * for CSCE 574 Homework 1
 * Spring 2023
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <tf/LinearMath/Quaternion.h> // Needed to convert rotation ...
#include <tf/LinearMath/Matrix3x3.h>  // ... quaternion into Euler angles
#include <math.h>
#include <tf/transform_listener.h>

/*struct Pose {
  double x; // in simulated Stage units
  double y; // in simulated Stage units
  double heading; // in radians
  ros::Time t; // last received time
  
  // Construct a default pose object with the time set to 1970-01-01
  Pose() : x(0), y(0), heading(0), t(0.0) {};
  
  // Process incoming pose message for current robot
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
      msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, heading);
    t = msg->header.stamp;
    ROS_INFO_STREAM("Pitch: " << pitch);
    ROS_INFO_STREAM("Roll: " << roll);
    //extra function
    //heading=tf::getYaw(msg->pose.pose.orientation);
  };
};

*/

class PotFieldBot {
public:
  // Construst a new Potential Field controller object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  PotFieldBot(ros::NodeHandle& nh , int robotID, int n, \
      double gx, double gy) : ID(robotID), numRobots(n), \
      goalX(gx), goalY(gy) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("base_scan", 1, \
      &PotFieldBot::laserCallback, this);
    
    // Subscribe to each robot' ground truth pose topic
    // and tell ROS to call pose->poseCallback(...) whenever a new
    // message is published on that topic
    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic "base_pose_ground_truth"
    poseSub = nh.subscribe("odom", 1, \
      &PotFieldBot::poseCallback, this);
  };


  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // TODO: parse laser data
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)
    float currentRange;
    /*int gamma = 1;
    int alpha = 1;
    int beta = 4;
    float epsilon = 0.05;
    float dsafe = 2.5;*/
    double MIN_SCAN_ANGLE_RAD = -90.0/180*M_PI;
    double MAX_SCAN_ANGLE_RAD = +90.0/180*M_PI;
    unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
    unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
    double angle = msg->angle_min;
    //ROS_INFO_STREAM("heading in laser: " << heading);
    float closestRange = msg->ranges[minIndex];
    double closestAngle;
    for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++){
      currentRange = msg->ranges[currIndex];
      angle += (msg->angle_increment);
      /*if(currentRange > dsafe+epsilon && currentRange < beta){
        force = -alpha/pow(currentRange-dsafe,2);
      }else if(currentRange < dsafe+epsilon){
        force = -alpha/pow(epsilon,2);
      }else{
        force = 0;
      }*/
      if (msg->ranges[currIndex] <closestRange) {
		closestRange = msg->ranges[currIndex];
		closestAngle = angle;
	  }
      //force_x = force_x + force * cos(angle+heading+M_PI);
      //force_y = force_y + force * sin(angle+heading+M_PI);
   
    }
    
    diffdistance = closestRange - desired_distance;
    
    diffangle = heading - closestAngle;
    
    /*ROS_INFO_STREAM("Repulsive Force_x: " << force_x);
    ROS_INFO_STREAM("Repulsive Force_y: " << force_y);
    
    
    xx = goalX - x;
    yy = goalY - y;
    force_Attractive = (xx*xx + yy*yy);        
             
    if(xx == 0 && yy >= 0 ){
      angle_temp = M_PI/2.0;
    }else if(xx == 0 && yy < 0){
      angle_temp = -M_PI/2.0;
    }else{
      angle_temp = atan2(yy,xx);
    }
    
    ROS_INFO_STREAM("Attractive force_x: " << force_Attractive * cos(angle_temp));
    ROS_INFO_STREAM("Attractive force_y: " << force_Attractive * sin(angle_temp));
     
    force_x += force_Attractive * cos(angle_temp);
    force_y += force_Attractive * sin(angle_temp);
        
    theta_diff = atan2(force_y, force_x);
    //ROS_INFO_STREAM("force_x: " << force_x);
    //ROS_INFO_STREAM("force_y: " << force_y);
    //simulated annealing
    /*if (abs(force_x) <= 0.05 && abs(force_y) <= 0.05){
       a = 2+rand()%2;
       ros::Time annealStartTime(ros::Time::now());
       ros::Duration annealDuration(a);
    
    }*/

    
  };

  // Process incoming ground truth robot pose message
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    heading=tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO_STREAM("ROBOT_POSE: " << heading);
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    ros::Rate rate(30); // Specify the FSM loop rate in Hz

    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove demo code, compute potential function, actuate robot
      
      // Demo code: print each robot's pose
      if(diffdistance == 0){
        move(FORWARD_SPEED_MPS, (M_PI/2-diffangle)/5);
      }else if(diffdistance > 0){;
        move(0, diffangle/5);
      }else{
        move(0, (M_PI-diffangle)/5);
      }

      /*if (abs(atan2(force_y,force_x) - heading) >= M_PI/2){
         move(0,(atan2(force_y,force_x) - heading)*0.25);
         ROS_INFO_STREAM("rotate");
         force_x = 0;
         force_y = 0;
         force = 0;
      }else if((xx != 0 || yy != 0) && fmod(atan(yy/xx) - heading, 2*M_PI)!=0){
         move(FORWARD_SPEED_MPS,(atan2(force_y,force_x) - heading)*0.25);
         ROS_INFO_STREAM("move&rotate");
         force_x = 0;
         force_y = 0;
         force = 0;
      }else if((xx != 0 || yy != 0) && fmod(atan(yy/xx) - heading,2*M_PI)==0){
         move(FORWARD_SPEED_MPS,0);
         ROS_INFO_STREAM("move");
         force_x = 0;
         force_y = 0;
         force = 0;
      }else{
         if (abs(xx) <= 0.5 && abs(yy) <= 0.5 && fmod(atan(yy/xx) - heading,2*M_PI)==0){
           move(0,0);
           ROS_INFO_STREAM("stop");
           force_x = 0;
           force_y = 0;
           force = 0;
         }
      }*/

      

      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
    }
  };
  
  

  constexpr static double FORWARD_SPEED_MPS = 0.5;
  constexpr static double ROTATE_SPEED_RADPS = M_PI/2;
  constexpr static float desired_distance = 0.5;
  
  
  
  protected:
    ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
    std::vector<ros::Subscriber> poseSubs; // List of subscribers to all robots' pose topics
    ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic
    //std::vector<Pose> pose; // List of pose objects for all robots
    int ID; // 0-indexed robot ID
    int numRobots; // Number of robots, positive value
    float diffangle;
    float diffdistance;
    double goalX, goalY; // Coordinates of goal
    double heading;
    double x,y;
    float force,force_x,force_y,force_Attractive; 
    float angle_temp;
    float xx,yy;
    float theta_diff;
    ros::Time annealStartTime; // Start time of the rotation
    ros::Duration annealDuration; // Duration of the rotation
    double a;
};


int main(int argc, char **argv) {
  int robotID = -1, numRobots = 0;
  double goalX, goalY;
  bool printUsage = false;
  //the initial force set as 0;
  
  
  // Parse and validate input arguments
  if (argc <= 4) {
    printUsage = true;
  } else {
    try {
      robotID = boost::lexical_cast<int>(argv[1]);
      numRobots = boost::lexical_cast<int>(argv[2]);
      goalX = boost::lexical_cast<double>(argv[3]);
      goalY = boost::lexical_cast<double>(argv[4]);

      if (robotID < 0) { printUsage = true; }
      if (numRobots <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [ROBOT_NUM_ID] [NUM_ROBOTS] [GOAL_X] [GOAL_Y]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "potfieldbot_" + std::string(argv[1])); // Initiate ROS node
  ros::NodeHandle n;
  PotFieldBot robbie(n, robotID, numRobots, goalX, goalY); // Create new random walk object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
