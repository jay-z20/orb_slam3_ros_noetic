/**
 *  @file initial_pose_server.hpp
 *  @brief Obtain the initial pose from .launch, .yaml or tfs 
 *         and sends it to the node that requests it.
 *  @package slamon
 *  @project SLAMON
 *
 *  @author
 *    Bo Miquel Nordfeldt-Fiol (2024–2025)
 *
 *  @license BSD-4-Clause
 *    This file is part of the SLAMON project and is released under
 *    the BSD 4-Clause License.
 */


#ifndef INITIAL_POSE_SERVER_HPP
#define INITIAL_POSE_SERVER_HPP


#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "orb_slam3_ros/GetTransform.h"


class InitialPoseServer
{


    public:

        InitialPoseServer();


    protected:

        bool getInitialPose(orb_slam3_ros::GetTransform::Request& req,
                            orb_slam3_ros::GetTransform::Response& res);


    private:

        std::string world_frame_; //!> World frame name.

        std::string robot_frame_; //!> Base link frame name.

        ros::NodeHandle nh_; //!> Public ROS node handle.

        ros::NodeHandle nhp_; //!> Private ROS node handle.

        ros::ServiceServer i_pose_server_; //!> Initial pose server.

        geometry_msgs::TransformStamped param_tf_; //!> Transform derived from the parameters.

        tf2_ros::Buffer tf_buffer_; //!> ROS transforms buffer.

        tf2_ros::TransformListener tf_listener_; //!> ROS transform listener.
};


#endif // INITIAL_POSE_SERVER_HPP