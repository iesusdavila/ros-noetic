//This program spawns a new turtlesim turtle by calling
// the appropriate service.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
//The srv class for the service.
#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>

ros::ServiceClient spawnClient;
ros::ServiceClient toggleForwardClient;
ros::Publisher pub;

bool createTurtle(turtlesim::Spawn::Request &req,
                  turtlesim::Spawn::Response &resp){

    ROS_INFO_STREAM("Creating a new turtle");
    req.x = 2;
    req.y = 3;
    req.theta = M_PI/2;
    req.name = "Leo";

    ros::service::waitForService("spawn", ros::Duration(5));
    bool success = spawnClient.call(req,resp);

    if(success){
	    ROS_INFO_STREAM("Spawned a turtle named "<< resp.name);
    }else{
	    ROS_ERROR_STREAM("Failed to spawn.");
    }
    return true;
}

bool toggleForward(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res){

    ROS_INFO_STREAM("Client toggling forward motion");

    ros::service::waitForService("toggle_forward", ros::Duration(5));
    bool success = toggleForwardClient.call(req,res);

    if(success){
	    ROS_INFO_STREAM("Toggle forward motion was correct");
    }else{
	    ROS_ERROR_STREAM("Toggle forward motion was incorrect");
    }
    return true;
}

void cmdvelMessageReceived(const geometry_msgs::Twist &msg) {
    double vel_x = msg.linear.x;
    double vel_y = msg.linear.y;
    double vel_w = msg.angular.z;
    
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed
                    << "x=" <<  vel_x 
                    << " y=" << vel_y
                    << " angular_z=" << vel_w);
    
    geometry_msgs::Twist msgLeo;
    msgLeo.linear.x = vel_x;
    msgLeo.linear.y = vel_y;
    msgLeo.angular.z = vel_w;

    pub.publish(msgLeo);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "spawn_turtle_plus");
    ros::NodeHandle nh;

    //Create a client object for the spawn service. This
    //needs to know the data type of the service and its name.
    spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");
    toggleForwardClient = nh.serviceClient<std_srvs::Empty>("toggle_forward");

    ros::Subscriber sub = nh.subscribe("turtle1/cmd_vel", 1000, &cmdvelMessageReceived);
    pub = nh.advertise<geometry_msgs::Twist>("Leo/cmd_vel",1000);

    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;
    createTurtle(req, resp);

    std_srvs::Empty::Request reqtoggleF;
	std_srvs::Empty::Response resptoggleF;
    toggleForward(reqtoggleF, resptoggleF);

    ros::spin();
}
