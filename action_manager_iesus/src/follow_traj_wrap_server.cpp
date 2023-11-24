#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <action_manager_iesus/set_point_to_point_trajectory.h>
#include <action_manager_iesus/set_goal_tolerances.h>
#include <action_manager_iesus/move_robot_trajectory.h>
#include <action_manager_iesus/move_robot_trajectory_safe.h>


//http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html
//http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

control_msgs::FollowJointTrajectoryGoal goal;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *robotClient;
bool cancelgoalactivated = false;
double xmin,xmax,ymin,ymax,zmin,zmax;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;
geometry_msgs::TransformStamped transformStamped;
std::vector<double> currentjoints;
bool updated = false;

//Callback function: Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("Estado terminado [%s]", state.toString().c_str());
  ROS_INFO("Resp: error_code es %d", result->error_code);
}

void activeCb()
{
  ROS_INFO("Goal just went active");
}

void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback: las posiciones actuales son (%f,%f,%f,%f,%f,%f)", feedback->actual.positions[0], feedback->actual.positions[1], feedback->actual.positions[2], feedback->actual.positions[3], feedback->actual.positions[4], feedback->actual.positions[5]);

  if(cancelgoalactivated){
    for(int i=0;i<3;i++){
        try{
            transformStamped = tfBuffer.lookupTransform("base_link","tool0", ros::Time(0));
        }
        catch (tf2::TransformException &ex){
            ros::Duration(0.5).sleep();
            continue;
        }

        if(transformStamped.transform.translation.x < xmin || transformStamped.transform.translation.x > xmax )
        {
            ROS_ERROR("Goal Cancelled:  position x %f esta fuera de los limites (%f,%f)",transformStamped.transform.translation.x, xmin,xmax);
            robotClient->cancelGoal();
        }
        else if(transformStamped.transform.translation.y < ymin || transformStamped.transform.translation.y > ymax )
        {
            ROS_ERROR("Goal Cancelled:  position y %f esta fuera de los limites (%f,%f)",transformStamped.transform.translation.y, ymin,ymax);
            robotClient->cancelGoal();
        }
        else if(transformStamped.transform.translation.z < zmin || transformStamped.transform.translation.z > zmax )
        {
            ROS_ERROR("Goal Cancelled:  position z %f esta fuera de los limites (%f,%f)",transformStamped.transform.translation.z, zmin,zmax);
            robotClient->cancelGoal();
        }
        break;
    }
  }
}

bool moveRobotTrajectory(
        action_manager_iesus::move_robot_trajectory::Request &req,
        action_manager_iesus::move_robot_trajectory::Response &resp)
{
  cancelgoalactivated = false;

  goal.trajectory.header.stamp = ros::Time::now();

  robotClient->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  bool finished_before_timeout = robotClient->waitForResult(ros::Duration(req.trajduration) + goal.goal_time_tolerance);

  actionlib::SimpleClientGoalState state = robotClient->getState();
  if (finished_before_timeout) {
      ROS_INFO("Accion del robot: %s  ",state.toString().c_str());
  } else {
      //Reports ACTIVE if not reached within timeout. Goal must be cancelled if we want to stop the motion, then the state will report PREEMPTED.
      ROS_ERROR("Accion del robot no finalizada: %s",state.toString().c_str());
      //Preempting task
      robotClient->cancelGoal();
  }
  return finished_before_timeout;
}

bool moveRobotTrajectorySafe(
        action_manager_iesus::move_robot_trajectory_safe::Request &req,
        action_manager_iesus::move_robot_trajectory_safe::Response &resp)
{

  cancelgoalactivated = true;
  xmin = req.limits[0];
  xmax = req.limits[1];
  ymin = req.limits[2];
  ymax = req.limits[3];
  zmin = req.limits[4];
  zmax = req.limits[5];

  goal.trajectory.header.stamp = ros::Time::now();
  robotClient->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  bool finished_before_timeout = robotClient->waitForResult(ros::Duration(req.trajduration) + goal.goal_time_tolerance);

  actionlib::SimpleClientGoalState state = robotClient->getState();
  if (finished_before_timeout) {
      ROS_INFO("Accion del robot culminado en: %s",state.toString().c_str());
  } else {
      ROS_ERROR("Accion del robot no culminado en: %s",state.toString().c_str());
      robotClient->cancelGoal();
  }
  return finished_before_timeout;
}

//Service to set the trajectory
bool setPointToPointTrajectory(
        action_manager_iesus::set_point_to_point_trajectory::Request &req,
        action_manager_iesus::set_point_to_point_trajectory::Response &resp)
{
        goal.trajectory.joint_names.resize( req.joint_names.size() );
        for(int i=0; i < req.joint_names.size(); i++){
            goal.trajectory.joint_names[i] = req.joint_names[i];
        }

        goal.trajectory.points.resize( 1 );

        uint vsize = req.goalPoint.positions.size();
        goal.trajectory.points[0].positions.resize( vsize );
        goal.trajectory.points[0].velocities.resize( vsize );
        goal.trajectory.points[0].accelerations.resize( vsize );
        for(int j=0; j < req.goalPoint.positions.size(); j++){
            goal.trajectory.points[0].positions[j] = req.goalPoint.positions[j];
            goal.trajectory.points[0].velocities[j] = 0.0;
            goal.trajectory.points[0].accelerations[j] = 0.0;
        }
        goal.trajectory.points[0].time_from_start = req.goalPoint.time_from_start;
  return true;
}


//Service to set the trajectory
bool setGoalTolerances(
        action_manager_iesus::set_goal_tolerances::Request &req,
        action_manager_iesus::set_goal_tolerances::Response &resp){

        goal.goal_tolerance.resize( req.jointGoalTolerances.size() );
        for(int i=0; i < req.jointGoalTolerances.size(); i++){
            goal.goal_tolerance[i].name = req.jointGoalTolerances[i].name;
            goal.goal_tolerance[i].position = req.jointGoalTolerances[i].position;
            goal.goal_tolerance[i].velocity = req.jointGoalTolerances[i].velocity;
            goal.goal_tolerance[i].acceleration = req.jointGoalTolerances[i].acceleration;
        }
        goal.goal_time_tolerance = req.goalTimeTolerance;

	return true;
}


int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "follow_traj_wrap_server");
  ros::NodeHandle nh;

  //Define simple action client and wail for server
  robotClient = new Client("/arm_controller/follow_joint_trajectory");
  if(!robotClient->waitForServer(ros::Duration(5.0)))
  {
      ROS_ERROR("Servidor no responde luego de los 5 segundos");
  };

  ros::ServiceServer server_settraj = nh.advertiseService("set_point_to_point_trajectory",&setPointToPointTrajectory);
  ros::ServiceServer server_setgoaltol = nh.advertiseService("set_goal_tolerances",&setGoalTolerances);
  ros::ServiceServer server_moverobottraj = nh.advertiseService("move_robot_trajectory",&moveRobotTrajectory);
  ros::ServiceServer server_moverobottrajsafe = nh.advertiseService("move_robot_trajectory_safe",&moveRobotTrajectorySafe);

  tfListener = new tf2_ros::TransformListener(tfBuffer);

  ros::Rate rate(1);
  while(ros::ok()) {
    // Wait until it's time for another iteration.
    ros::spinOnce();
    rate.sleep();
  }

}