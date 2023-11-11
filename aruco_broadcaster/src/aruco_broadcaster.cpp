/* *************************************************************************\
  Copyright 2022-2023 Institute of Industrial and Control Engineering (IOC)
                Universitat Politecnica de Catalunya
                BarcelonaTech
* Software License Agreement (BSD License)
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Institute of Industrial and Control Engineering,
*     (IOC), Universitat Politecnica de Catalunya or BarcelonaTech nor
*     the names of its contributors may be used to endorse or promote
*     products derived from this software without specific prior
*     written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Leopold Palomo-Avellaneda
  Desc:   aruco broadcaster new version
*/


#include <iostream>
#include <vector>
#include <cstdint>

#include <algorithm>

// --> ROS Dependencies included HERE
#include <ros/ros.h>
#include <tf2/transform_storage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

// aruco ros includes
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>

// generated include
#include <aruco_broadcaster/getMarkerTf.h>

class aruco_broadcaster_class{
    private:

        ros::NodeHandle nh;
        std::string nodename;

        // --> Global variables defined HERE
        ros::Publisher markers_pub;
        ros::Subscriber camera_aruco_tf_subs;

        std::vector<int> markerList;
        std::string camera_frame, aruco_frame;
        bool publish_all;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        tf2_ros::TransformBroadcaster tfBroadcaster;

        ros::ServiceServer getMarkerTfserver;



    public:

        aruco_broadcaster_class(std::string name) : tfListener(tfBuffer)  {
        // in this way, the tfBuffer initialices tfListener
        // using private - namespace aruco_broadcaster -
            // --> Global variables initialized HERE

            // configure stuff
            // we need some parameters
            nodename = name;
            nh = ros::NodeHandle(nodename);
            // checking the params about the list
            if (nh.hasParam("markerList"))
            {
              nh.getParam("markerList", markerList);
              if (markerList.empty())
              {
                ROS_WARN_STREAM("Aruco broadcaster will publish all the arucos found");
                publish_all = true;
              }
              else
              {
                 ROS_WARN_STREAM("markerList has size of: " << markerList.size() << std::endl);
                 publish_all = false;
              }
            }
            else
            {
              ROS_INFO_STREAM("No param named markerList");
              publish_all = true;
            }

            if (nh.hasParam("camera_frame"))
            {
              nh.getParam("camera_frame", camera_frame);
              if(camera_frame.empty())
              {
                ROS_FATAL_STREAM("Parameter camera frame has not reference. The program will exit!!!");
                exit(1);
              }
              else
              {
                ROS_WARN_STREAM("Parameter camera_frame named " + camera_frame);
              }
            }
            else
            {
              ROS_INFO_STREAM("No param given in camera_frame");
            }

            if (nh.hasParam("aruco_frame"))
            {
              nh.getParam("aruco_frame", aruco_frame);
              if (aruco_frame.empty())
              {
                ROS_FATAL_STREAM("Parameter aruco_frame is empty. The program will exit!!!");
                exit(1);
              }
              else
              {
                ROS_WARN_STREAM("Parameter aruco_frame is named " + aruco_frame);
              }
            }
            else
            {
              ROS_INFO_STREAM("No param given in aruco_frame");
            }

            camera_aruco_tf_subs = nh.subscribe("/aruco_marker_publisher/markers", 1, &aruco_broadcaster_class::arucoCallback, this);

            getMarkerTfserver = nh.advertiseService("get_marker_Tf",&aruco_broadcaster_class::getMarkerTf, this);

        }

        // --> Functions and Callbacks defined HERE
        void arucoCallback(const aruco_msgs::MarkerArray &marker_info)
        {
            tf2::Transform _cameraToMarker, _CameraToReference, _referenceToMarker;
            geometry_msgs::TransformStamped transformStamped;

            if(publish_all)
            {
                for( uint i = 0; i < marker_info.markers.size();i++)
                {
                    tf2::fromMsg(marker_info.markers.at(i).pose.pose, _cameraToMarker);
                    transformStamped.header.stamp = ros::Time::now();
                    transformStamped.header.frame_id = camera_frame;
                    transformStamped.child_frame_id = aruco_frame + "_" + std::to_string(marker_info.markers.at(i).id);
                    transformStamped.transform = tf2::toMsg(_cameraToMarker);
                    tfBroadcaster.sendTransform(transformStamped);
                }
            }
            else
            {
                for (unsigned int i = 0; i < marker_info.markers.size(); ++i)
                {
                    if(std::find(markerList.begin(), markerList.end(), marker_info.markers.at(i).id) != markerList.end())
                    {
                        tf2::fromMsg(marker_info.markers.at(i).pose.pose, _cameraToMarker);
                        transformStamped.header.stamp = ros::Time::now();
                        transformStamped.header.frame_id = camera_frame;
                        transformStamped.child_frame_id = aruco_frame + "_" + std::to_string(marker_info.markers.at(i).id);
                        transformStamped.transform = tf2::toMsg(_cameraToMarker);
                        tfBroadcaster.sendTransform(transformStamped);
                    }
                }
             }
        }

        bool getMarkerTf(aruco_broadcaster::getMarkerTf::Request &req,
                         aruco_broadcaster::getMarkerTf::Response &resp)
        {
            bool result;
            try {
                resp.ret = getTF(req.parent, req.markerId, result);
                ROS_DEBUG_STREAM("result := " << result << std::endl);

            }  catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
            }
            return result;
        }
        geometry_msgs::TransformStamped getTF(const std::string& parent, const std::string& child, bool &result)
        {
            geometry_msgs::TransformStamped transformStamped;
            result = false;

            try{
                    transformStamped = tfBuffer.lookupTransform(parent, child, ros::Time(0));
                    result = true;
             }
             catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(0.1).sleep();
                    result = false;
            }
            return transformStamped;
        }

        // Transform data types
        void tfBroadcast(const std::string& parent,
                         const std::string& child,
                         const tf2::Transform& tf2_trasnf)
        {
            geometry_msgs::TransformStamped transform_msg;
            tf2::Stamped<tf2::Transform> tf2_stamped(
                tf2_trasnf, ros::Time::now(), parent.c_str());
            tf2::convert(tf2_stamped, transform_msg);
            transform_msg.child_frame_id=child;
            tfBroadcaster.sendTransform(transform_msg);
        }

        void printTransform(const std::string& name, const tf2::Transform& pose)
        {
            double x, y, z;
            tf2Scalar yaw, pitch, roll;
            x = pose.getOrigin().getX();
            y = pose.getOrigin().getY();
            z = pose.getOrigin().getZ();
            tf2::Matrix3x3 mat(pose.getRotation());
            mat.getRPY(roll, pitch, yaw);
            ROS_INFO("Transform %s: [%f, %f, %f, %f, %f, %f]",
                    name.c_str(), x, y, z, roll, pitch, yaw);
        }

        
        // main function to run
        void run()
        {
            ros::Rate rate(15);
            while (ros::ok()){

                ROS_INFO_ONCE("\033[1;32m---->\033[0m Aruco_broadcaster is running.");
                // do some job
                // Tell ROS to spin the callbacks
                ros::spinOnce();
                // sleep
                rate.sleep();
            }
        }
};

int main(int argc, char** argv){

    // This must be called before anything else ROS-related
    ros::init(argc, argv, "aruco_broadcaster");
    std::string name = ros::this_node::getName();
    aruco_broadcaster_class node(name);

    ROS_INFO("\033[1;32m---->\033[0m Starting MyNode.");

    node.run();

    return 0;
}
