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
 *  \file	CartesianCommander.h
 *  \author	Toshio Ueshiba
 *  \brief	ROS node for tracking corners in 2D images
 */
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/transform_listener.h>
#include <aist_cartesian_commander/TrackWithContactAction.h>

namespace aist_cartesian_commander
{
/************************************************************************
*  class CartesianCommander						*
************************************************************************/
class CartesianCommander
{
  private:
    using pose_t	  = geometry_msgs::PoseStamped;
    using pose_cp	  = geometry_msgs::PoseStampedConstPtr;
    using twist_t	  = geometry_msgs::TwistStamped;
    using twist_cp	  = geometry_msgs::TwistStampedConstPtr;
    using wrench_t	  = geometry_msgs::WrenchStamped;
    using transform_t	  = geometry_msgs::TransformStamped;
    using transform_cp	  = geometry_msgs::TransformStampedConstPtr;

    using pose_pub_t	  = realtime_tools::RealtimePublisher<pose_t>;
    using pose_pub_p	  = std::shared_ptr<pose_pub_t>;
    using wrench_pub_t	  = realtime_tools::RealtimePublisher<wrench_t>;
    using wrench_pub_p	  = std::shared_ptr<wrench_pub_t>;
    using action_server_t = actionlib::SimpleActionServer<
				TrackWithContactAction>;
    using sync_t	  = message_filters::TimeSynchronizer<pose_t, twist_t>;

  public:
		CartesianCommander(ros::NodeHandle& nh,
				   const std::string& nodelet_name)	;

  private:
    const std::string&
		getName()		const	{ return _nodelet_name; }

    void	goal_cb()						;
    void	preempt_cb()						;
    void	pose_cb(const pose_cp& target_pose)			;
    void	controller_state_cb(const pose_cp&  current_pose,
				    const twist_cp& current_twist)	;

  private:
    const std::string				_nodelet_name;

    ros::Subscriber				_target_pose_sub;
    message_filters::Subscriber<pose_t>		_current_pose_sub;
    message_filters::Subscriber<twist_t>	_current_twist_sub;
    sync_t					_sync;

    const pose_pub_p				_target_frame_pub;
    const wrench_pub_p				_target_wrench_pub;

    action_server_t				_track_srv;
    TrackWithContactGoalConstPtr		_current_goal;

    tf2_ros::Buffer				_tf2_buffer;
    const tf2_ros::TransformListener		_listener;

    pose_t					_current_pose;
    twist_t					_current_twist;
    bool					_ready;
    mutable std::mutex				_mtx;

    transform_cp				_Tet;
    wrench_t					_target_wrench;
};

}	// namespace aist_cartesian_commander
