#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "urdf_tutorial/changescale.h"
#include "urdf_tutorial/changecontrolledjoints.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>


double deltaPan;
double deltaTilt;
double scale;
double danger_factor;
const double degree2rad = M_PI/180;
int controlled_by_pan;
int controlled_by_tilt;

bool changescale(urdf_tutorial::changescale::Request  &req,
                urdf_tutorial::changescale::Response &res)
{
    scale = req.scale;
    ROS_INFO("Scale changed to = %f", req.scale);
    return true;
}

bool changecontrolledjoints(urdf_tutorial::changecontrolledjoints::Request  &req,
                            urdf_tutorial::changecontrolledjoints::Response &res)
{
    controlled_by_pan = req.c1;
    controlled_by_tilt = req.c2;
    return true;
}


void valuesCallback(const geometry_msgs::Twist& msg)
{
    deltaPan = msg.linear.x * scale;
    deltaTilt = msg.angular.z * scale;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_publisher");
    ros::NodeHandle n;

    //The node subscribes to the values given by the keys in the keyboard
    ros::Subscriber sub = n.subscribe("teleop_values", 1, valuesCallback);

    //The node advertises the joint values of the pan-tilt
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    //The node provides a service to change the scale factor of the teleoperated motions
    ros::ServiceServer serviceScale = n.advertiseService("change_scale", changescale);

    //The node provides a service to change the joints being controlled
    ros::ServiceServer serviceJoints = n.advertiseService("change_controlled_joints", changecontrolledjoints);

    ros::Rate loop_rate(50);


    // message declarations
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(9);
    joint_state.position.resize(9);
    double pan = 0.0;
    double tilt = 0.0;

    deltaPan = 0.0;
    deltaTilt = 0.0;
    scale = 0.5;
    danger_factor = 0.1;

    double limit_inf[9];
    double limit_sup[9];

    limit_inf[0] = -150*degree2rad;
    limit_inf[1] = -67*degree2rad;
    limit_inf[2] = -150*degree2rad;
    limit_inf[3] = -92*degree2rad;
    limit_inf[4] = -150*degree2rad;
    limit_inf[5] = -92*degree2rad;
    limit_inf[6] = -150*degree2rad;
    limit_inf[7] =   0*mm2m;
    limit_inf[8] = -25*mm2m;

    limit_sup[0] = 114*degree2rad;
    limit_sup[1] = 114*degree2rad;
    limit_sup[2] = 41*degree2rad;
    limit_sup[3] = 110*degree2rad;
    limit_sup[4] = 150*degree2rad;
    limit_sup[5] = 113*degree2rad;
    limit_sup[6] = 150*degree2rad;
    limit_sup[7] = 25*mm2m;
    limit_sup[8] = 0*mm2m;

    controlled_by_pan = 3;
    controlled_by_tilt = 5;

    //initial joint values  
    joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    joint_state.position[2] = 0;
    joint_state.position[3] = 0;
    joint_state.position[4] = 0;
    joint_state.position[5] = 0;
    joint_state.position[6] = 0;
    joint_state.position[7] = 0;
    joint_state.position[8] = 0;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;

    while (ros::ok())
    {
        //listen to the teleop_keys
        ros::spinOnce();

        try{
            transformStamped = tfBuffer.lookupTransform("base_link", "grasping_frame", ros::Time(0), ros::Duration(0.1));
        }
        catch (tf2::TransformException ex ){
            joint_state.header.stamp = ros::Time::now();
            joint_state.name[0] ="shoulder_pan_joint";
            joint_state.name[1] ="shoulder_pitch_joint";
            joint_state.name[2] ="elbow_roll_joint";
            joint_state.name[3] ="elbow_pitch_joint";
            joint_state.name[4] ="wrist_roll_joint";
            joint_state.name[5] ="wrist_pitch_joint";
            joint_state.name[6] ="gripper_roll_joint";
            joint_state.name[7] ="finger_joint1";
            joint_state.name[8] ="finger_joint2";
            joint_pub.publish(joint_state);
            loop_rate.sleep();
            continue;
        }

        pan = joint_state.position[controlled_by_pan];
        tilt = joint_state.position[controlled_by_tilt];

        if(transformStamped.transform.translation.z < 0.3 && transformStamped.transform.translation.z!=0.0)
        {
            ROS_INFO("Applying danger_factor reduction. Tool at z-value = %f", transformStamped.transform.translation.z);
            deltaPan *= danger_factor;
            deltaTilt *= danger_factor;
        }

        if (pan+deltaPan < limit_sup[controlled_by_pan] && pan+deltaPan > limit_inf[controlled_by_pan]) pan = pan + deltaPan;
        if (tilt+deltaTilt < limit_sup[controlled_by_tilt] && tilt+deltaTilt >  limit_inf[controlled_by_tilt] ) tilt = tilt + deltaTilt;

        //update joint_state
        joint_state.position[controlled_by_pan] = pan;
        joint_state.position[controlled_by_tilt] = tilt;

        joint_state.header.stamp = ros::Time::now();
        joint_state.name[0] ="shoulder_pan_joint";
        joint_state.name[1] ="shoulder_pitch_joint";
        joint_state.name[2] ="elbow_roll_joint";
        joint_state.name[3] ="elbow_pitch_joint";
        joint_state.name[4] ="wrist_roll_joint";
        joint_state.name[5] ="wrist_pitch_joint";
        joint_state.name[6] ="gripper_roll_joint";
        joint_state.name[7] ="finger_joint1";
        joint_state.name[8] ="finger_joint2";

        //send the joint state
        joint_pub.publish(joint_state);

        deltaPan=0;
        deltaTilt=0;

        loop_rate.sleep();
    }
    return 0;
}
