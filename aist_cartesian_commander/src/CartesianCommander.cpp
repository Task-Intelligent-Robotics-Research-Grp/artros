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
#include "CartesianCommander.h"

namespace aist_cartesian_commander
{
/************************************************************************
*  static functions							*
************************************************************************/

/************************************************************************
*  class CartesianCommander						*
************************************************************************/
CartesianCommander::CartesianCommander(ros::NodeHandle& nh,
				       const std::string& nodelet_name)
    :_nodelet_name(nodelet_name),
     _target_twist_sub(nh.subscribe<twist_t>("/target_twist", 1,
					     &CartesianCommander::twist_cb,
					     this)),
     _current_pose_sub( nh, "/current_pose",  1),
     _current_twist_sub(nh, "/current_twist", 1),
     _sync(_current_pose_sub, _current_twist_sub, 1),
     _target_frame_pub( nh.advertise<pose_t>(  "target_frame",  1)),
     _target_wrench_pub(nh.advertise<wrench_t>("target_wrench", 1)),
     _track_srv(nh, "track_with_contact", false),
     _current_goal(nullptr),
     _tf2_buffer(),
     _listener(_tf2_buffer),
     _ddr(nh),
     _control_period(0.002),
     _current_pose(nullptr),
     _current_twist(nullptr),
     _mtx(),
     _target_frame(),
     _target_wrench()
{
  // Setup frames for pose and wrench commands
    _target_frame.header.frame_id  = nh.param<std::string>("robot_base_link",
							   "base_link");
    _target_wrench.header.frame_id = nh.param<std::string>("end_effector_link",
							   "tool0");

    _sync.registerCallback(&CartesianCommander::controller_state_cb, this);

  // Setup and start TrackWithContact action server
    _track_srv.registerGoalCallback(boost::bind(&CartesianCommander::goal_cb,
						this));
    _track_srv.registerPreemptCallback(boost::bind(
					   &CartesianCommander::preempt_cb,
					   this));
    _track_srv.start();

  // Setup ddynamic_reconfigure server
    _ddr.registerVariable<double>("control_period", &_control_period,
				  "Period for integrating twist(sec)",
				  0.001, 0.05);
    _ddr.publishServicesTopicsAndUpdateConfigData();
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
    _current_pose  = nullptr;
    _current_twist = nullptr;
    _Tb		   = nullptr;
}

void
CartesianCommander::preempt_cb()
{
    _track_srv.setPreempted();
    NODELET_WARN_STREAM("(cartesian_commander) goal CANCELLED");
}

void
CartesianCommander::twist_cb(const twist_cp& target_twist)
{
    const std::lock_guard<std::mutex>	lock(_mtx);

    if (!_track_srv.isActive() || !_current_pose || !_current_twist)
	return;

    if (!_Tb)
    {
	try
	{
	    _Tb = boost::make_shared<transform_t>(
		      _tf2_buffer.lookupTransform(
			  _target_frame.header.frame_id,
			  target_twist->header.frame_id,
			  ros::Time(0), ros::Duration(1.0)));
	    _tf2_buffer.transform(_current_goal->target_wrench, _target_wrench,
				  _target_wrench.header.frame_id,
				  ros::Duration(1.0));
	}
	catch (const std::exception& err)
	{
	    _track_srv.setAborted();
	    NODELET_ERROR_STREAM("(cartesian_commander) goal ABORTED["
				 << err.what() << ']');
	    return;
	}
    }

    twist_t	twist;
    tf2::doTransform(*target_twist, twist, *_Tb);


    _target_frame.header.stamp = target_twist->header.stamp;
    _target_frame_pub.publish(_target_frame);
    _target_wrench.header.stamp = target_twist->header.stamp;
    _target_wrench_pub.publish(_target_wrench);

}

void
CartesianCommander::controller_state_cb(const pose_cp&  current_pose,
					const twist_cp& current_twist)
{
    const std::lock_guard<std::mutex>	lock(_mtx);

    _current_pose  = current_pose;
    _current_twist = current_twist;

    // std::cerr << current_pose->header.frame_id << ", "
    // 	      << current_twist->header.frame_id << std::endl;
}

}	// namespace aist_cartesian_commander
