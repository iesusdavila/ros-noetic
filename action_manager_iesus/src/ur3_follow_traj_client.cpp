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
  ros::init(argc, argv, "ur3_follow_traj_client");
  ros::NodeHandle nh;

  ros::ServiceClient client_settraj = nh.serviceClient<action_manager_iesus::set_point_to_point_trajectory>("set_point_to_point_trajectory");
  ros::ServiceClient client_setgoaltol = nh.serviceClient<action_manager_iesus::set_goal_tolerances>("set_goal_tolerances");
  ros::ServiceClient client_moverobottraj = nh.serviceClient<action_manager_iesus::move_robot_trajectory>("move_robot_trajectory");
  ros::ServiceClient client_moverobottrajsafe = nh.serviceClient<action_manager_iesus::move_robot_trajectory_safe>("move_robot_trajectory_safe");

  action_manager_iesus::set_point_to_point_trajectory::Request reqsetraj;
  action_manager_iesus::set_point_to_point_trajectory::Response respsettraj;

  double moveduration = 10.0;

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

  ros::service::waitForService("set_point_to_point_trajectory", ros::Duration(5));

  action_manager_iesus::move_robot_trajectory::Request reqmovetraj;
  action_manager_iesus::move_robot_trajectory::Response respmovetraj;
  reqmovetraj.trajduration = moveduration;
  ros::service::waitForService("move_robot_trajectory", ros::Duration(5));


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


  std::cout<<"\nPRESS UNA TECLA PARA CONTINUAR: "<<std::endl;
  std::cin.get();

  //ros::Rate rate(1/cycletime);
  char c='u';
  while(ros::ok()) {
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
    else
    {
        reqsetraj.goalPoint.positions[0] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[1] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[2] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[3] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[4] = -1.5 + 3.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[5] = -1.5 + 3.0*rand()/RAND_MAX;
        std::cout<<"Configuracion de movimiento ("<<reqsetraj.goalPoint.positions[0]<<","
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
    std::cout<<"\nPRESIONA F PARA ACABAR TODO"<<std::endl;
    std::cin >> c;
  }
}
