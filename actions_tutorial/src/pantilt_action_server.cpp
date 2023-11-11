/*
 * Copyright (C) 2015, Lentin Joseph and Qbotics Labs Inc.
 * Email id : qboticslabs@gmail.com
 *
 * Copyright (C) 2017, Jonathan Cacace.
 * Email id : jonathan.cacace@gmail.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//Modifications of the demo_action_server.cpp, done by Jan Rosell, 2018.
//Moves a selected joint of the pan-tilt strcture accoring to the goal, and keeps publishing the joint values.
//The goal is set as joint_value (in degrees) and joint_name.
//The maximum allowed time to reach the goal is set in seconds.

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <actionlib/server/simple_action_server.h>
#include "actions_tutorial/pantilt_actionAction.h"
#include <iostream>
#include <sstream>
#include <sensor_msgs/JointState.h>


const double degree2rad = M_PI/180;
sensor_msgs::JointState joint_state;

class pantilt_actionAction
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<actions_tutorial::pantilt_actionAction> as;
  // create messages that are used to published feedback/result
  actions_tutorial::pantilt_actionFeedback feedback;
  actions_tutorial::pantilt_actionResult result;

  std::string action_name;
  double currentvalue;
  double scale;

public:
  pantilt_actionAction(std::string name) :
    as(nh_, name, boost::bind(&pantilt_actionAction::executeCB, this, boost::placeholders::_1), false),
    action_name(name)
  {
	as.registerPreemptCallback(boost::bind(&pantilt_actionAction::preemptCB, this));
        as.start();
        scale=1;
  }

  ~pantilt_actionAction(void)
  {
  }

  void preemptCB()
  {
    ROS_WARN("%s got preempted!", action_name.c_str());
    result.final_count = currentvalue;
    as.setPreempted(result,"I got Preempted");
  }
  void executeCB(const actions_tutorial::pantilt_actionGoalConstPtr &goal)
  {
    if(!as.isActive() || as.isPreemptRequested()) return;

    ROS_INFO("%s is processing the goal %f for joint %s", action_name.c_str(), goal->count, goal->name.c_str());
    
    //Set the joint to be controlled
    int jointnumber;
    std::string jointname;
    if(goal->name=="pan")
    {
      jointnumber=0;
      jointname = "pan_joint";
    }
    else
    {
      jointnumber=1;
      jointname = "tilt_joint";
    }

    //detrmine the motion direction
    currentvalue = joint_state.position[jointnumber];
    int sign;
    if(currentvalue-goal->count < 0)
        sign=1;
    else
        sign=-1;

    //loop: keep increasing/decreasing the joint value until the goal has been reached or timeout.
    ros::Rate rate(10);
    while(ros::ok())
    {
      //update joint_state - moving one degree*scale
      joint_state.name[jointnumber] = jointname;
      joint_state.position[jointnumber] = currentvalue + sign * degree2rad * scale;
      currentvalue = joint_state.position[jointnumber];

       if(!as.isActive() || as.isPreemptRequested())
      {
        return;
      }

      //ROS_INFO("%f %f",abs(currentvalue-goal->count),degree2rad * scale);
      if(fabs(currentvalue-goal->count)< degree2rad * scale)
      {
        ROS_INFO("%s Succeeded at getting joint %s  to goal %f", action_name.c_str(), goal->name.c_str(), goal->count);
        result.final_count = currentvalue;
        as.setSucceeded(result);
      }
      else
      {
        ROS_INFO("Setting to goal %f / %f",feedback.current_number,goal->count);
        feedback.current_number = currentvalue;
        as.publishFeedback(feedback);
      }

      rate.sleep();
    }
    // not ros::ok()
    result.final_count = currentvalue;
    as.setAborted(result,"I failed !");
    ROS_INFO("%s Shutting down",action_name.c_str());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pantilt_action_server");
  ros::NodeHandle nh;
  ROS_INFO("Starting pantilt Action Server");
  pantilt_actionAction pantilt_action_obj(ros::this_node::getName());

  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.header.stamp = ros::Time::now();
  joint_state.name[0] ="pan_joint";
  joint_state.position[0] = 0.0;
  joint_state.name[1] ="tilt_joint";
  joint_state.position[1] = 0.0;

  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Rate loop_rate(30);//set equal to the loop_rate in the CB
  while (ros::ok())
  {
      //update joint_state time stamp - joint values changes in the CB
      joint_state.header.stamp = ros::Time::now();
      joint_pub.publish(joint_state);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
