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
 *  \file	CartesianCommander.cpp
 *  \author	Toshio UESHIBA
 *  \brief	ROS node for tracking corners in 2D images
 */
#include <nodelet/nodelet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "CartesianCommander.h"

namespace aist_cartesian_commander
{
/************************************************************************
*  static functions							*
************************************************************************/
static geometry_msgs::Pose
update_pose(const geometry_msgs::Pose& pose, const geometry_msgs::Pose& delta,
	    const geometry_msgs::Transform& transform)
{
    tf2::Transform	tf2_pose;
    fromMsg(pose, tf2_pose);
    tf2::Transform	tf2_delta;
    fromMsg(delta, tf2_delta);
    tf2::Transform	tf2_transform;
    fromMsg(transform, tf2_transform);

    tf2_pose *= tf2_transform * tf2_delta * tf2_transform.inverse();

    geometry_msgs::Pose	updated_pose;
    toMsg(tf2_pose, updated_pose);

    return updated_pose;
}

/************************************************************************
*  class CartesianCommander						*
************************************************************************/
CartesianCommander::CartesianCommander(ros::NodeHandle& nh,
				       const std::string& nodelet_name)
    :_nodelet_name(nodelet_name),
     _target_pose_sub(nh.subscribe<pose_t>("/target_pose", 1,
					   &CartesianCommander::pose_cb,
					   this)),
     _current_pose_sub( nh, "/current_pose",  1),
     _current_twist_sub(nh, "/current_twist", 1),
     _sync(_current_pose_sub, _current_twist_sub, 1),
     _target_frame_pub(new pose_pub_t(nh, "target_frame",  1)),
     _target_wrench_pub(new wrench_pub_t(nh, "target_wrench", 1)),
     _track_srv(nh, "track_with_contact", false),
     _current_goal(nullptr),
     _tf2_buffer(),
     _listener(_tf2_buffer),
     _current_pose(),
     _current_twist(),
     _ready(false),
     _mtx(),
     _Tet(nullptr),
     _target_wrench()
{
  // Setup frame_id for pose and wrench commands.
    _target_wrench.header.frame_id = nh.param<std::string>("end_effector_link",
							   "tool0");

  // Setup and start TrackWithContact action server
    _track_srv.registerGoalCallback(boost::bind(&CartesianCommander::goal_cb,
						this));
    _track_srv.registerPreemptCallback(boost::bind(
					   &CartesianCommander::preempt_cb,
					   this));
    _track_srv.start();

  // Start subscribing current pose and twist of the robot.
    _sync.registerCallback(&CartesianCommander::controller_state_cb, this);
}

void
CartesianCommander::goal_cb()
{
    _current_goal = _track_srv.acceptNewGoal();
    NODELET_INFO_STREAM("(cartesian_commander) goal ACCEPTED");

    if (_track_srv.isPreemptRequested())
    {
	preempt_cb();
	return;
    }

    const std::lock_guard<std::mutex>	lock(_mtx);
    _ready = false;
    _Tet   = nullptr;
}

void
CartesianCommander::preempt_cb()
{
    const std::lock_guard<std::mutex>	lock(_mtx);

    TrackWithContactResult		result;
    result.current_pose  = _current_pose;
    result.current_twist = _current_twist;
    _track_srv.setPreempted(result);

    NODELET_WARN_STREAM("(cartesian_commander) goal CANCELLED");
}

void
CartesianCommander::pose_cb(const pose_cp& target_pose)
{
    const std::lock_guard<std::mutex>	lock(_mtx);

    if (!_track_srv.isActive() || !_ready)
	return;

    if (!_Tet)
    {
	try
	{
	  // Get a transform from the frame describing incoming pose
	  // to the end-effector link of the robot.
	    _Tet = boost::make_shared<transform_t>(
		       _tf2_buffer.lookupTransform(
			   _target_wrench.header.frame_id,	// ee-link
			   target_pose->header.frame_id,
			   ros::Time(0), ros::Duration(1.0)));

	  // Transform the wrench value given in the goal to the one
	  // w.r.t. the end-effector link.
	    _tf2_buffer.transform(_current_goal->target_wrench, _target_wrench,
				  _target_wrench.header.frame_id,
				  ros::Duration(1.0));
	}
	catch (const std::exception& err)
	{
	    TrackWithContactResult	result;
	    result.current_pose  = _current_pose;
	    result.current_twist = _current_twist;
	    _track_srv.setAborted(result);

	    NODELET_ERROR_STREAM("(cartesian_commander) goal ABORTED["
				 << err.what() << ']');
	    return;
	}
    }

  // Update current pose by incoming twist and publish.
    pose_t	target_frame;
    target_frame.pose = update_pose(_current_pose.pose, target_pose->pose,
				    _Tet->transform);
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

  // Publish feedback.
    TrackWithContactFeedback	feedback;
    feedback.current_pose  = _current_pose;
    feedback.current_twist = _current_twist;
    _track_srv.publishFeedback(feedback);
}

void
CartesianCommander::controller_state_cb(const pose_cp&  current_pose,
					const twist_cp& current_twist)
{
    const std::lock_guard<std::mutex>	lock(_mtx);

    _current_pose  = *current_pose;
    _current_twist = *current_twist;
    _ready = true;
}
}	// namespace aist_cartesian_commander
