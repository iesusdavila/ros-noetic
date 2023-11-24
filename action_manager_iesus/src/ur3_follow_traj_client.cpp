#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <action_manager_iesus/set_point_to_point_trajectory.h>
#include <action_manager_iesus/set_goal_tolerances.h>
#include <action_manager_iesus/move_robot_trajectory.h>
#include <action_manager_iesus/move_robot_trajectory_safe.h>


int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "ur3_follow_traj_client");
  ros::NodeHandle nh;

  //Create client objects for the services.
  ros::ServiceClient client_settraj = nh.serviceClient<action_manager_iesus::set_point_to_point_trajectory>("set_point_to_point_trajectory");
  ros::ServiceClient client_setgoaltol = nh.serviceClient<action_manager_iesus::set_goal_tolerances>("set_goal_tolerances");
  ros::ServiceClient client_moverobottraj = nh.serviceClient<action_manager_iesus::move_robot_trajectory>("move_robot_trajectory");
  ros::ServiceClient client_moverobottrajsafe = nh.serviceClient<action_manager_iesus::move_robot_trajectory_safe>("move_robot_trajectory_safe");

  ///////////////////////////////////////////
  //Create the request and response objects for setting the traj
  action_manager_iesus::set_point_to_point_trajectory::Request reqsetraj;
  action_manager_iesus::set_point_to_point_trajectory::Response respsettraj;

  //Set the trajectory to follow
  double moveduration = 10.0;

  //Set point to point trajectory
  reqsetraj.joint_names.resize(6);
  reqsetraj.joint_names[0] = "shoulder_pan_joint";
  reqsetraj.joint_names[1] = "shoulder_lift_joint";
  reqsetraj.joint_names[2] = "elbow_joint";
  reqsetraj.joint_names[3] = "wrist_1_joint";
  reqsetraj.joint_names[4] = "wrist_2_joint";
  reqsetraj.joint_names[5] = "wrist_3_joint";

  reqsetraj.goalPoint.positions.resize(6);
  reqsetraj.goalPoint.positions[0] = 0.0;
  reqsetraj.goalPoint.positions[1] = 0.0;
  reqsetraj.goalPoint.positions[2] = 0.0;
  reqsetraj.goalPoint.positions[3] = 0.0;
  reqsetraj.goalPoint.positions[4] = 0.0;
  reqsetraj.goalPoint.positions[5] = 0.0;
  reqsetraj.goalPoint.time_from_start = ros::Duration(moveduration);

  //call the set_trajectory service
  ros::service::waitForService("set_point_to_point_trajectory", ros::Duration(5));

  ///////////////////////////////////////////
  //Create the request and response objects for moving the robot
  action_manager_iesus::move_robot_trajectory::Request reqmovetraj;
  action_manager_iesus::move_robot_trajectory::Response respmovetraj;
  reqmovetraj.trajduration = moveduration;
  ros::service::waitForService("move_robot_trajectory", ros::Duration(5));


  ///////////////////////////////////////////
  //Create the request and response objects for moving the robot with limits
  action_manager_iesus::move_robot_trajectory_safe::Request reqmovetrajsafe;
  action_manager_iesus::move_robot_trajectory_safe::Response respmovetrajsafe;
  reqmovetrajsafe.trajduration = moveduration;
  reqmovetrajsafe.limits.resize(6);
  reqmovetrajsafe.limits[0] = -100000.0; //xmin
  reqmovetrajsafe.limits[1] =  100000.0; //xmax
  reqmovetrajsafe.limits[2] = -100000.0; //ymin
  reqmovetrajsafe.limits[3] =  100000.0; //ymax
  reqmovetrajsafe.limits[4] = 0.1; //zmin
  reqmovetrajsafe.limits[5] =  1000000.0; //zmax
  ros::service::waitForService("move_robot_trajectorysafe", ros::Duration(5));


  //Call function to send goal
  //Wait for user to press a key
  std::cout<<"\nPRESS A KEY TO START..."<<std::endl;
  std::cin.get();

  //ros::Rate rate(1/cycletime);
  char c='u';
  while(ros::ok()) {
    ///////////////////////////////////////////
    //Send the goal to move the robot
    //move unconstrained
    if(c!='c'){
        reqsetraj.goalPoint.positions[0] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[1] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[2] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[3] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[4] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[5] = -1.5 + 3.0*rand()/RAND_MAX;
        std::cout<<"Free motion to configuration ("<<reqsetraj.goalPoint.positions[0]<<", "
          <<reqsetraj.goalPoint.positions[1]<<")"<<", "
          <<reqsetraj.goalPoint.positions[2]<<")"<<", "
          <<reqsetraj.goalPoint.positions[3]<<")"<<", "
          <<reqsetraj.goalPoint.positions[4]<<")"<<", "
          <<reqsetraj.goalPoint.positions[5]<<")"<<std::endl;
        client_settraj.call(reqsetraj,respsettraj);
        client_moverobottraj.call(reqmovetraj,respmovetraj);
    }
    //move contrained (preemptrs if out of cartesian limits)
    else
    {
        reqsetraj.goalPoint.positions[0] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[1] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[2] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[3] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[4] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[5] = -1.5 + 3.0*rand()/RAND_MAX;
        std::cout<<"Contrained motion to configuration ("<<reqsetraj.goalPoint.positions[0]<<","
          <<reqsetraj.goalPoint.positions[1]<<")"<<", "
          <<reqsetraj.goalPoint.positions[2]<<")"<<", "
          <<reqsetraj.goalPoint.positions[3]<<")"<<", "
          <<reqsetraj.goalPoint.positions[4]<<")"<<", "
          <<reqsetraj.goalPoint.positions[5]<<") - preempts if out of cartesian limits x["
          <<reqmovetrajsafe.limits[0]<<","<<reqmovetrajsafe.limits[1]<<"],y["
          <<reqmovetrajsafe.limits[2]<<","<<reqmovetrajsafe.limits[3]<<"],z["
          <<reqmovetrajsafe.limits[4]<<","<<reqmovetrajsafe.limits[5]<<"]"<<std::endl;
        client_settraj.call(reqsetraj,respsettraj);
        client_moverobottrajsafe.call(reqmovetrajsafe,respmovetrajsafe);
    }
    //Wait for user to press a key
    std::cout<<"\nPRESS (f) for a free motion or PRESS (c) for a constrained motion..."<<std::endl;
    //std::cin.get();
    std::cin >> c;

    // Wait until it's time for another iteration.
    //rate.sleep();
  }
}
