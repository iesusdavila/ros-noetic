#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <actions_tutorial/set_point_to_point_trajectory.h>
#include <actions_tutorial/set_goal_tolerances.h>
#include <actions_tutorial/move_robot_trajectory.h>
//Uncomment next line if you have implemented the motion along a safe trajectory (i.e. within limits)
//#include <actions_tutorial/move_robot_trajectory_safe.h>


int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "pantilt_follow_traj_client");
  ros::NodeHandle nh;

  //Create client objects for the services.
  ros::ServiceClient client_settraj = nh.serviceClient<actions_tutorial::set_point_to_point_trajectory>("set_point_to_point_trajectory");
  ros::ServiceClient client_setgoaltol = nh.serviceClient<actions_tutorial::set_goal_tolerances>("set_goal_tolerances");
  ros::ServiceClient client_moverobottraj = nh.serviceClient<actions_tutorial::move_robot_trajectory>("move_robot_trajectory");
  
  //Uncomment next line if you have implemented the motion along a safe trajectory (i.e. within limits)
  //ros::ServiceClient client_moverobottrajsafe = nh.serviceClient<actions_tutorial::move_robot_trajectory_safe>("move_robot_trajectory_safe");

  ///////////////////////////////////////////
  //Create the request and response objects for setting the traj
  actions_tutorial::set_point_to_point_trajectory::Request reqsetraj;
  actions_tutorial::set_point_to_point_trajectory::Response respsettraj;

  //Set the trajectory to follow
  //Defines a sinoidal trajectory for the tilt joint and fixed value for the pan joint
  double moveduration = 10.0;//3 seconds

  //Set point to point trajectory
  reqsetraj.joint_names.resize(2);
  reqsetraj.joint_names[0] = "pan_joint";
  reqsetraj.joint_names[1] = "tilt_joint";

  reqsetraj.goalPoint.positions.resize(2);
  reqsetraj.goalPoint.positions[0] = 0.0;
  reqsetraj.goalPoint.positions[1] = 0.0;
  reqsetraj.goalPoint.time_from_start = ros::Duration(moveduration);

  //call the set_trajectory service
  ros::service::waitForService("set_point_to_point_trajectory", ros::Duration(5));


  ///////////////////////////////////////////
  //Create the request and response objects for setting the goal tolerances
  actions_tutorial::set_goal_tolerances::Request reqgoaltol;
  actions_tutorial::set_goal_tolerances::Response respgoaltol;

  //Set the goal tolerance parameters
  reqgoaltol.jointGoalTolerances.resize(2);
  reqgoaltol.jointGoalTolerances[0].name = "pan_joint";
  reqgoaltol.jointGoalTolerances[0].position = 0.1;
  reqgoaltol.jointGoalTolerances[0].velocity = 0.1;
  //jointGoalTolerances[0].acceleration = 0.6;
  reqgoaltol.jointGoalTolerances[1].name = "tilt_joint";
  reqgoaltol.jointGoalTolerances[1].position = 0.1;
  reqgoaltol.jointGoalTolerances[1].velocity = 0.1;
  //jointGoalTolerances[1].acceleration = 0.6;
  reqgoaltol.goalTimeTolerance = ros::Duration(1.0);

  //call the set_trajectory service
  ros::service::waitForService("set_goal_tolerances", ros::Duration(5));
  client_setgoaltol.call(reqgoaltol,respgoaltol);


  ///////////////////////////////////////////
  //Create the request and response objects for moving the robot
  actions_tutorial::move_robot_trajectory::Request reqmovetraj;
  actions_tutorial::move_robot_trajectory::Response respmovetraj;
  reqmovetraj.trajduration = moveduration;
  ros::service::waitForService("move_robot_trajectory", ros::Duration(5));


  //Uncomment next lines if you have implemented the motion along a safe trajectory (i.e. within limits)
  /*
  ///////////////////////////////////////////
  //Create the request and response objects for moving the robot with limits
  actions_tutorial::move_robot_trajectory_safe::Request reqmovetrajsafe;
  actions_tutorial::move_robot_trajectory_safe::Response respmovetrajsafe;
  reqmovetrajsafe.trajduration = moveduration;
  reqmovetrajsafe.limits.resize(6);
  reqmovetrajsafe.limits[0] = -100000.0; //xmin
  reqmovetrajsafe.limits[1] =  100000.0; //xmax
  reqmovetrajsafe.limits[2] = -100000.0; //ymin
  reqmovetrajsafe.limits[3] =  100000.0; //ymax
  reqmovetrajsafe.limits[4] = -100000.0; //zmin
  reqmovetrajsafe.limits[5] =  0.3; //zmax
  ros::service::waitForService("move_robot_trajectorysafe", ros::Duration(5));
 */

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
        reqsetraj.goalPoint.positions[0] = -0.5 + 1.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[1] = -3.5 + 1.0*rand()/RAND_MAX;
        std::cout<<"Free motion to configuration ("<<reqsetraj.goalPoint.positions[0]<<","
          <<reqsetraj.goalPoint.positions[1]<<")"<<std::endl;
        client_settraj.call(reqsetraj,respsettraj);
        client_moverobottraj.call(reqmovetraj,respmovetraj);
    }
    //move contrained (preemptrs if out of cartesian limits)
    else
    {
        //Uncomment next line if you have implemented the motion along a safe trajectory (i.e. within limits)
        /*
        reqsetraj.goalPoint.positions[0] = -0.5 + 1.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[1] = -3.5 + 1.0*rand()/RAND_MAX;
        std::cout<<"Contrained motion to configuration ("<<reqsetraj.goalPoint.positions[0]<<","
            <<reqsetraj.goalPoint.positions[1]<<") - preempts if out of cartesian limits x["
            <<reqmovetrajsafe.limits[0]<<","<<reqmovetrajsafe.limits[1]<<"],y["
            <<reqmovetrajsafe.limits[2]<<","<<reqmovetrajsafe.limits[3]<<"],z["
            <<reqmovetrajsafe.limits[4]<<","<<reqmovetrajsafe.limits[5]<<"]"<<std::endl;
        client_settraj.call(reqsetraj,respsettraj);
        client_moverobottrajsafe.call(reqmovetrajsafe,respmovetrajsafe);
        */
    }
    //Wait for user to press a key
    std::cout<<"\nPRESS (f) for a free motion or PRESS (c) for a constrained motion..."<<std::endl;
    //std::cin.get();
    std::cin >> c;

    // Wait until it's time for another iteration.
    //rate.sleep();
  }
}
