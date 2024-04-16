#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>                    // Needed for rand()
#include <ctime>                      // Needed to seed random number genator with a time value
#include <tf/LinearMath/Quaternion.h> // Needed to convert rotation ...
#include <tf/LinearMath/Matrix3x3.h>  // ... quaternion into Euler angles
#include <tf/transform_listener.h>

using namespace std;
struct Pose {
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
  };
};

class PotFieldBot
{
public:
    // Construst a new Potential Field controller object and hook up
    // this ROS node to the simulated robot's pose, velocity control,
    // and laser topics
    PotFieldBot(ros::NodeHandle &nh, int robotID, int n, double gx, double gy)
        : ID(robotID), numRobots(n), goalX(gx), goalY(gy)
    {
        // Initialize random time generator
        srand(time(NULL));

        // Advertise a new publisher for the current simulated robot's
        // velocity command topic (the second argument indicates that
        // if multiple command messages are in the queue to be sent,
        // only the last command will be sent)
        commandPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

        // Subscribe to the current simulated robot's laser scan topic and
        // tell ROS to call this->laserCallback() whenever a new message
        // is published on that topic
        laserSub = nh.subscribe("/scan", 1, &PotFieldBot::laserCallback, this);
	poseSub = nh.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 10, &PotFieldBot::poseCallback, this);
       // poseSub = nh.subscribe("base_pose_ground_truth", 1, &PotFieldBot::poseCallback, this);

    };

    // Process incoming ground truth robot pose message
    void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // cout << "posecallback" << endl;
        double roll, pitch;
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;

        heading = tf::getYaw(msg->pose.pose.orientation);
    };

    // Send a velocity command
    void move(double linearVelMPS, double angularVelRadPS)
    {
        geometry_msgs::Twist msg; // The default constructor will set all commands to 0
        msg.linear.x = linearVelMPS;
        msg.angular.z = angularVelRadPS;
        commandPub.publish(msg);
    };

  double RepulsiveForceX(double d, double angle) {
    if (d < (PROXIMITY_RANGE_M + EPSILON)) {
        double repulsiveForce = ALPHA / (EPSILON * EPSILON);
        return -repulsiveForce * cos(angle);
    } else if (d < BETA) {
        double bs = (d - PROXIMITY_RANGE_M) * (d - PROXIMITY_RANGE_M);
        double repulsiveForce = ALPHA / bs;
        return -repulsiveForce * cos(angle);
    } else {
        return 0;
    }
  }
  
  double RepulsiveForceY(double d, double angle) {
    if (d < (PROXIMITY_RANGE_M + EPSILON)) {
        double repulsiveForce = ALPHA / (EPSILON * EPSILON);
        return -repulsiveForce * sin(angle);
    } else if (d < BETA) {
        double bs = (d - PROXIMITY_RANGE_M) * (d - PROXIMITY_RANGE_M);
        double repulsiveForce = ALPHA / bs;
        return -repulsiveForce * sin(angle);
    } else {
        return 0;
    }
  }

  double AttractiveForceX(double x1, double y1) {
    double dx = x1 - x;
    double dy = y1 - y;
    double d = sqrt(dx * dx + dy * dy);

    return dx * d;
  }
  
  double AttractiveForceY(double x1, double y1) {
    double dx = x1 - x;
    double dy = y1 - y;
    double d = sqrt(dx * dx + dy * dy);

    return dy * d;
  }
  bool achieve_goal()
    {
        if (abs(x - goalX) < 0.3 && abs(y - goalY) < 0.3)
        {
            return true;
        }
        return false;
    }

    double generateRandomForce() {
        return (rand() % 200 - 100) / 100.0; 
    }


    double StayingNearSamePlace() {

        double distance_traveled = sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));

        prevX = x;
        prevY = y;

        if (distance_traveled < STAYING_NEAR_SAME_PLACE_THRESHOLD) {
            staying_near_same_place_counter += 1;
        } else {
            staying_near_same_place_counter = 0;
        }

        return (staying_near_same_place_counter);
    }


    // Process incoming laser scan message
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        //  TODO: parse laser data
        //  (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)

        // Upon receiving the laserscan data, the robot takes 2 actions:
        // 1. turn to the calculated direction
        // 2. move forward towards that direction


        unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
        unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);


        double forceX = AttractiveForceX(goalX, goalY);
        double forceY = AttractiveForceY(goalX, goalY);
        

        for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++)
        {
            double d = msg->ranges[currIndex];
            double angle = msg->angle_min + currIndex * msg->angle_increment + heading;
            forceX += RepulsiveForceX(d, angle);
            forceY += RepulsiveForceY(d, angle);
        }
        
        
        int staying_near_same_place_counter = StayingNearSamePlace();
        if (staying_near_same_place_counter >= STAYING_NEAR_SAME_PLACE_DURATION) {
            RandomForceMag = staying_near_same_place_counter;// * RandomForceFactor;
            ROS_INFO("StayingNearSamePlace");
            ROS_INFO_STREAM("staying_near_same_place_counter: "<< staying_near_same_place_counter);
        }
        else {
            RandomForceMag = 0;
        }
        
        ROS_INFO_STREAM("RandomForceMag: "<< RandomForceMag);
        
        double random_force_x = generateRandomForce() * RandomForceMag;
        double random_force_y = generateRandomForce() * RandomForceMag;
        
        ROS_INFO_STREAM("RandomForceMag: "<< RandomForceMag);
        ROS_INFO_STREAM("generateRandomForce: "<< generateRandomForce());
        ROS_INFO_STREAM("random_force_x: "<< random_force_x);
        ROS_INFO_STREAM("random_force_y: "<< random_force_y);
        
        forceX += random_force_x;
        forceY += random_force_y;       
        
        
        ROS_INFO_STREAM("force_x: "<< forceX);
        ROS_INFO_STREAM("force_y: "<< forceY);
        
        targetDirection = atan2(forceY, forceX);
        ROS_INFO_STREAM("targetDirection: "<< targetDirection);
        
        double ForceAngle = targetDirection - heading;
        
        if (ForceAngle > M_PI) {
            ForceAngle = ForceAngle - 2 * M_PI;
        }
        if (ForceAngle < -M_PI) {
            ForceAngle = 2 * M_PI + ForceAngle;
        }

        rotate_speed_radps = KAPPA * ForceAngle;


    }

    // Main FSM loop for ensuring that ROS messages are
    // processed in a timely manner, and also for sending
    // velocity controls to the simulated robot based on the FSM state
    void spin()
    {
        ros::Rate rate(30); // Specify the FSM loop rate in Hz

        while (ros::ok())
        {
            if (!achieve_goal())
            {
                move(forward_speed_mps, rotate_speed_radps);
            }

            ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
            rate.sleep();    // Sleep for the rest of the cycle, to enforce the FSM loop rate
        }
    };

    // Tunable motion controller parameters
    double forward_speed_mps = 0.5;
    double rotate_speed_radps = M_PI / 4;
    int staying_near_same_place_counter = 0;
    double random_force_x = 0.0;
    double random_force_y = 0.0;
        
    double prevX = 0;
    double prevY = 0;
    double RandomForce = 1;  
   // double staying_near_same_place_counter = 0;  
    static constexpr double MIN_SCAN_ANGLE_RAD = -30.0 / 180 * M_PI;
    static constexpr double MAX_SCAN_ANGLE_RAD = 30.0 / 180 * M_PI;
    static constexpr double PROXIMITY_RANGE_M = 1;

    // Only care about obstacles that is in a range of 4 meters
    static constexpr double BETA = 4.0;
    static constexpr double EPSILON = 0.05;
    static constexpr double GAMMA = 2.5;
    static constexpr double KAPPA = 0.3;
    static constexpr double ALPHA = 1;
    static constexpr double ANGLE_EPSILON = 0.1;
    static constexpr double RandomForceFactor = 2;
    static constexpr double STAYING_NEAR_SAME_PLACE_THRESHOLD = 0.05; 
    static constexpr int STAYING_NEAR_SAME_PLACE_DURATION = 3; 

protected:
    ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
    ros::Subscriber laserSub;  // Subscriber to the current robot's laser scan topic
    ros::Subscriber poseSub;   // Subscriber to current robots' pose topic
 //   std::vector<ros::Subscriber> poseSub; // List of subscribers to all robots' pose topics
 //   std::vector<Pose> pose; // List of pose objects for all robots
    double x;
    double y;
    double heading;
    double targetDirection;
    double RandomForceMag;
    
    int ID;              // 0-indexed robot ID
    int numRobots;       // Number of robots, positive value
    double goalX, goalY; // Coordinates of goal
};

int main(int argc, char **argv)
{
    int robotID = -1, numRobots = 0;
    double goalX, goalY;
    bool printUsage = false;

    // Parse and validate input arguments
    if (argc <= 4)
    {
        printUsage = true;
    }
    else
    {
        try
        {
            robotID = boost::lexical_cast<int>(argv[1]);
            numRobots = boost::lexical_cast<int>(argv[2]);
            goalX = boost::lexical_cast<double>(argv[3]);
            goalY = boost::lexical_cast<double>(argv[4]);

            if (robotID < 0)
            {
                printUsage = true;
            }
            if (numRobots <= 0)
            {
                printUsage = true;
            }
        }
        catch (std::exception err)
        {
            printUsage = true;
        }
    }
    if (printUsage)
    {
        std::cout << "Usage: " << argv[0] << " [ROBOT_NUM_ID] [NUM_ROBOTS] [GOAL_X] [GOAL_Y]" << std::endl;
        return EXIT_FAILURE;
    }

    ros::init(argc, argv, "potfieldbot");                    // Initiate ROS node
    ros::NodeHandle n;                                       // Create named handle "robot_#"
    PotFieldBot robbie(n, robotID, numRobots, goalX, goalY); // Create new random walk object
    robbie.spin();                                           // Execute FSM loop

    return EXIT_SUCCESS;
};
