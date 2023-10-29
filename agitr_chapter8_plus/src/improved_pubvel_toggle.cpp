//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <agitr_chapter8_plus/Changerate.h>
#include <agitr_chapter8_plus/Changevelocity.h>

bool start = true;
double newvelocity;
bool velchanged = false;
double newfrequency;
bool ratechanged = false;

bool toggleStart(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        start = !start;
        ROS_INFO_STREAM("Now sending "<<(start?
                        "start":"stop")<< " commands.");
	return true;
}

bool changeVelocity(
        agitr_chapter8_plus::Changevelocity::Request &req,
        agitr_chapter8_plus::Changevelocity::Response &resp){
        ROS_INFO_STREAM("Changing velocity to "<<req.newvelocity);

        newfrequency = req.newvelocity;
        velchanged = true;
        return true;
}

bool changeRate(
        agitr_chapter8_plus::Changerate::Request &req,
        agitr_chapter8_plus::Changerate::Response &resp){

        ROS_INFO_STREAM("Changing rate to "<<req.newrate);

        newfrequency = req.newrate;
        ratechanged = true;
        return true;
}


int main(int argc, char **argv){
        ros::init(argc,argv,"improved_pubvel_toggle_start");
	ros::NodeHandle nh;
        
	ros::ServiceServer server = 
		nh.advertiseService("toggle_start",&toggleStart);

        ros::ServiceServer server1 =
                nh.advertiseService("change_velocity",&changeVelocity);
                
        ros::ServiceServer server0 =
                nh.advertiseService("change_activate_turtlesim",&changeRate);
                
        ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);
    
        ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
                msg.linear.x = start?1.0:0.0;
                msg.angular.z = start?0.0:0.0;

                if(velchanged) {
                    msg.linear.x = newfrequency;
                    velchanged = false;
                }

		pub.publish(msg);
		ros::spinOnce();

                if(ratechanged) {
                    rate = ros::Rate(newfrequency);
                    ratechanged = false;
                }

		rate.sleep();
	}
}
