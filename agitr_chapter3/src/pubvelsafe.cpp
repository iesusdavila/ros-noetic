#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <time.h>

// Define the safe zone as a square around the center of the window
const float SAFE_ZONE_SIZE = 8.0;
const float SAFE_ZONE_X_MIN = 5.0 - SAFE_ZONE_SIZE / 2.0;
const float SAFE_ZONE_X_MAX = 5.0 + SAFE_ZONE_SIZE / 2.0;
const float SAFE_ZONE_Y_MIN = 5.0 - SAFE_ZONE_SIZE / 2.0;
const float SAFE_ZONE_Y_MAX = 5.0 + SAFE_ZONE_SIZE / 2.0;

// Define the fixed linear velocity inside the safe zone
const float FIXED_LINEAR_VEL = 1.0;

// Define the callback function for the turtle's pose
void poseCallback(const turtlesim::Pose::ConstPtr& poseMsg, ros::Publisher& velPub)
{
    // Get the current position of the turtle
    float x = poseMsg->x;
    float y = poseMsg->y;

    // Check if the turtle is inside the safe zone
    if (x >= SAFE_ZONE_X_MIN && x <= SAFE_ZONE_X_MAX && y >= SAFE_ZONE_Y_MIN && y <= SAFE_ZONE_Y_MAX)
    {
        ROS_INFO_STREAM("Zone secureted, sending velocity command equal to " << FIXED_LINEAR_VEL << "");
        // Set a fixed linear velocity and a random angular velocity
        geometry_msgs::Twist velMsg;
        velMsg.linear.x = FIXED_LINEAR_VEL;
        velMsg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
        velPub.publish(velMsg);
        ROS_INFO_STREAM("Sending velocity command:"
                        << " linear=" << velMsg.linear.x
                        << " angular=" << velMsg.angular.z);
    }
    else
    {
        ROS_INFO_STREAM("Zone insecureted, sending random velocity command");
        // Set a random linear and angular velocity
        geometry_msgs::Twist velMsg;
        velMsg.linear.x = double(rand())/double(RAND_MAX);
        velMsg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
        velPub.publish(velMsg);
        ROS_INFO_STREAM("Sending random velocity command:"
                        << " linear=" << velMsg.linear.x
                        << " angular=" << velMsg.angular.z);
    }
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "pubvelsafe");
    ros::NodeHandle nh;

    // Seed the random number generator
    srand(time(0));

    // Create a publisher for the turtle's velocity
    ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 2000);

    // Create a subscriber for the turtle's pose
    ros::Subscriber poseSub = nh.subscribe<turtlesim::Pose>("turtle1/pose", 1000, boost::bind(poseCallback, _1, velPub));

    // Spin the node
    ros::spin();

    return 0;
}
