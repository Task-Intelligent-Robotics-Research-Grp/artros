// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba
//
/*!
 *  \file	TwistIntegrator.cpp
 *  \author	Toshio UESHIBA
 *  \brief	ROS node for tracking corners in 2D images
 */
#include <nodelet/nodelet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "TwistIntegrator.h"

namespace aist_twist_integrator
{
/************************************************************************
*  static functions							*
************************************************************************/
static geometry_msgs::Twist
transform_twist(const geometry_msgs::Twist& twist,
		const geometry_msgs::Transform& transform)
{
    tf2::Vector3	tf2_linear;
    tf2::fromMsg(twist.linear, tf2_linear);
    tf2::Vector3	tf2_angular;
    tf2::fromMsg(twist.angular, tf2_angular);
    tf2::Transform	tf2_transform;
    tf2::fromMsg(transform, tf2_transform);

    tf2_angular	= tf2_transform(tf2_angular);
    tf2_linear	= tf2_transform(tf2_linear)
		+ tf2::tf2Cross(tf2_transform.getOrigin(), tf2_angular);

    geometry_msgs::Twist	transformed_twist;
    transformed_twist.linear  = tf2::toMsg(tf2_linear);
    transformed_twist.angular = tf2::toMsg(tf2_angular);

    return transformed_twist;
}

static geometry_msgs::Pose
update_pose(const geometry_msgs::Pose& pose,
	    const geometry_msgs::Twist& twist, tf2Scalar dt)
{
    tf2::Transform	tf2_pose;
    tf2::fromMsg(pose, tf2_pose);
    const tf2::Vector3	tf2_axis(twist.angular.x * dt,
				 twist.angular.y * dt,
				 twist.angular.z * dt);
    tf2_pose *= tf2::Transform(tf2::Quaternion(tf2_axis, tf2_axis.length()),
			       tf2::Vector3(twist.linear.x * dt,
					    twist.linear.y * dt,
					    twist.linear.z * dt));

    geometry_msgs::Pose	updated_pose;
    tf2::toMsg(tf2_pose, updated_pose);

    return updated_pose;
}


/************************************************************************
*  class TwistIntegrator						*
************************************************************************/
TwistIntegrator::TwistIntegrator(ros::NodeHandle& nh,
				       const std::string& nodelet_name)
    :_nodelet_name(nodelet_name),
     _target_twist_sub(nh.subscribe<twist_t>("/target_twist", 1,
					     &TwistIntegrator::twist_cb,
					     this)),
     _current_pose_sub( nh, "/current_pose",  1),
     _current_twist_sub(nh, "/current_twist", 1),
     _sync(_current_pose_sub, _current_twist_sub, 1),
     _target_frame_pub(new pose_pub_t(nh, "target_frame",  1)),
     _target_wrench_pub(new wrench_pub_t(nh, "target_wrench", 1)),
     _tf2_buffer(),
     _listener(_tf2_buffer),
     _ddr(nh),
     _control_period(0.002),
     _current_pose(),
     _current_twist(),
     _mtx(),
     _Tet(nullptr),
     _target_wrench()
{
  // Setup frame_id for pose and wrench commands.
    _target_wrench.header.frame_id = nh.param<std::string>("end_effector_link",
							   "tool0");

  // Setup ddynamic_reconfigure server
    _ddr.registerVariable<double>("control_period", &_control_period,
				  "Period for integrating twist(sec)",
				  0.001, 0.05);
    _ddr.publishServicesTopicsAndUpdateConfigData();

  // Start subscribing current pose and twist of the robot.
    _sync.registerCallback(&TwistIntegrator::controller_state_cb, this);
}

void
TwistIntegrator::twist_cb(const twist_cp& target_twist)
{
    const std::lock_guard<std::mutex>	lock(_mtx);

    if (!_Tet)
    {
	try
	{
	  // Get a transform from the frame describing incoming twist
	  // to the end-effector link of the robot.
	    _Tet = boost::make_shared<transform_t>(
		       _tf2_buffer.lookupTransform(
			   _target_wrench.header.frame_id,
			   target_twist->header.frame_id,
			   ros::Time(0), ros::Duration(1.0)));
	}
	catch (const std::exception& err)
	{
	    NODELET_ERROR_STREAM("(twist_integrator) goal ABORTED["
				 << err.what() << ']');
	    return;
	}
    }

  // Update current pose by incoming twist and publish.
    pose_t	target_frame;
    target_frame.pose = update_pose(_current_pose.pose,
				    transform_twist(target_twist->twist,
						    _Tet->transform),
				    _control_period);
    target_frame.header.frame_id = _current_pose.header.frame_id;
    target_frame.header.stamp    = ros::Time::now();
    if (_target_frame_pub->trylock())
    {
	_target_frame_pub->msg_ = target_frame;
	_target_frame_pub->unlockAndPublish();
    }

  // Publish desired wrench.
    _target_wrench.header.stamp = target_frame.header.stamp;
    if (_target_wrench_pub->trylock())
    {
	_target_wrench_pub->msg_ = _target_wrench;
	_target_wrench_pub->unlockAndPublish();
    }
}

void
TwistIntegrator::controller_state_cb(const pose_cp&  current_pose,
				     const twist_cp& current_twist)
{
    const std::lock_guard<std::mutex>	lock(_mtx);

    _current_pose  = *current_pose;
    _current_twist = *current_twist;
}

}	// namespace aist_twist_integrator
