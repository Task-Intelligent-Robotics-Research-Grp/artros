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
  \file	 Calibrator.cpp
  \brief Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <sstream>
#include <ctime>
#include <cstdlib>	// for std::getenv()
#include <sys/stat.h>	// for mkdir()
#include <errno.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <aist_msgs/srv/get_calibration_sample_list.hpp>
#include <aist_msgs/srv/compute_calibration.hpp>
#include <aist_msgs/srv/take_calibration_sample.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <aist_utility/geometry_msgs.hpp>
#include <aist_utility/fileio.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include "HandEyeCalibration.h"

namespace aist_handeye_calibration
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
class Calibrator : public rclcpp::Node
{
  private:
    template <class MSG>
    using sub_p		= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using srv_p		= typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_p		= typename SRV::Request::SharedPtr;
    template <class SRV>
    using res_p		= typename SRV::Response::SharedPtr;

    using transform_t		= geometry_msgs::msg::TransformStamped;
    using pose_t		= geometry_msgs::msg::PoseStamped;
    using pose_cp		= pose_t::ConstSharedPtr;
    using get_sample_list_t	= aist_msgs::srv::GetCalibrationSampleList;
    using compute_calibration_t	= aist_msgs::srv::ComputeCalibration;
    using trigger_t		= std_srvs::srv::Trigger;
    using empty_t		= std_srvs::srv::Empty;
    using take_sample_t		= aist_msgs::srv::TakeCalibrationSample;

  public:
		Calibrator(const rclcpp::NodeOptions& options)		;

  private:
    const std::string&	camera_frame()				const	;
    const std::string&	effector_frame()			const	;
    const std::string&	marker_frame()				const	;
    const std::string&	world_frame()				const	;

    void	pose_cb(pose_cp pose)					;
    void	take_sample(const req_p<take_sample_t>,
			    const res_p<take_sample_t> res)		;
    void	get_sample_list(const req_p<get_sample_list_t>,
				const res_p<get_sample_list_t> res)	;
    void	compute_calibration(const req_p<compute_calibration_t>,
				    const res_p<compute_calibration_t> res);
    void	reset(const req_p<empty_t>, const res_p<empty_t>)	;

  private:
    const sub_p<pose_t>			_pose_sub;
    const srv_p<take_sample_t>		_take_sample_srv;
    const srv_p<get_sample_list_t>	_get_sample_list_srv;
    const srv_p<compute_calibration_t>	_compute_calibration_srv;
    const srv_p<empty_t>		_reset_srv;

    pose_cp				_pose;
    std::mutex				_pose_mtx;
    std::condition_variable		_pose_cv;
    const std::chrono::duration<double>	_pose_timeout;

    tf2_ros::Buffer			_tf2_buffer;
    const tf2_ros::TransformListener	_tf2_listener;

    std::vector<transform_t>	_Tcm;	//!< in:  camera <- marker   transform
    std::vector<transform_t>	_Twe;	//!< in:  world  <- effector transform
    transform_t			_Tec;	//!< out: effector <- camera transform
    transform_t			_Twm;	//!< out: world    <- marker transform

    const bool			_use_dual_quaternion;
    const bool			_eye_on_hand;
    const std::string		_camera_name;
};

Calibrator::Calibrator(const rclcpp::NodeOptions& options)
    :rclcpp::Node("calibrator", options),
     _pose_sub(create_subscription<pose_t>("pose", 1,
					   std::bind(&Calibrator::pose_cb,
						     this,
						     std::placeholders::_1))),
     _take_sample_srv(create_service<take_sample_t>(
			  "~/take_sample",
			  std::bind(&Calibrator::take_sample, this,
				    std::placeholders::_1,
				    std::placeholders::_2))),
     _get_sample_list_srv(create_service<get_sample_list_t>(
			      "~/get_sample_list",
			      std::bind(&Calibrator::get_sample_list, this,
					std::placeholders::_1,
					std::placeholders::_2))),
     _compute_calibration_srv(create_service<compute_calibration_t>(
				  "~/compute_calibration",
				  std::bind(&Calibrator::compute_calibration,
					    this,
					    std::placeholders::_1,
					    std::placeholders::_2))),
     _reset_srv(create_service<empty_t>("~/reset",
					std::bind(&Calibrator::reset, this,
						  std::placeholders::_1,
						  std::placeholders::_2))),
     _pose(nullptr),
     _pose_mtx(),
     _pose_cv(),
     _pose_timeout(ddynamic_reconfigure2::declare_read_only_parameter(
		       this, "pose_timeout", 3.0)),
     _tf2_buffer(get_clock()),
     _tf2_listener(_tf2_buffer),
     _use_dual_quaternion(ddynamic_reconfigure2::declare_read_only_parameter(
			      this, "use_dual_quaternion", true)),
     _eye_on_hand(ddynamic_reconfigure2::declare_read_only_parameter(
		      this, "eye_on_hand", true)),
     _camera_name(ddynamic_reconfigure2::declare_read_only_parameter(
		      this, "camera_name", "camera"))
{
    if (_eye_on_hand)
    {
	_Tec.header.frame_id = ddynamic_reconfigure2::
			       declare_read_only_parameter(
				   this, "end_effector_link", "tool0");
	_Twm.header.frame_id = ddynamic_reconfigure2::
			       declare_read_only_parameter(
				   this, "reference_frame", "base_link");
    }
    else
    {
	_Twm.header.frame_id = ddynamic_reconfigure2::
			       declare_read_only_parameter(
				   this, "end_effector_link", "tool0");
	_Tec.header.frame_id = ddynamic_reconfigure2::
			       declare_read_only_parameter(
				   this, "reference_frame", "base_link");
    }

    _Tec.child_frame_id = "";
    _Twm.child_frame_id = ddynamic_reconfigure2::declare_read_only_parameter(
			      this, "marker_frame", "marker_frame");

    RCLCPP_INFO_STREAM(get_logger(), "calibrator initialized");
}

const std::string&
Calibrator::camera_frame() const
{
    return _Tec.child_frame_id;
}

const std::string&
Calibrator::effector_frame() const
{
    return _Tec.header.frame_id;
}

const std::string&
Calibrator::marker_frame() const
{
    return _Twm.child_frame_id;
}

const std::string&
Calibrator::world_frame() const
{
    return _Twm.header.frame_id;
}

void
Calibrator::pose_cb(pose_cp pose)
{
    const std::lock_guard<std::mutex>	lock(_pose_mtx);

    _pose = pose;
    _pose_cv.notify_all();
}

void
Calibrator::take_sample(const req_p<take_sample_t>,
			const res_p<take_sample_t> res)
{
    using	namespace aist_utility;

    RCLCPP_INFO_STREAM(get_logger(),
		       "new request for TakeCalibrationSample received");

    std::unique_lock<std::mutex>	lock(_pose_mtx);

    _pose = nullptr;

  // Get the latest pose of the marker.
    if (!_pose_cv.wait_for(lock, _pose_timeout,
			   [this]{ return _pose != nullptr; }))
    {
	res->success = false;
	res->message = "timeout[" + std::to_string(_pose_timeout.count())
		     + "sec] has expired before marker pose available";
	return;
    }

  // Set camera frame.
    _Tec.child_frame_id = _pose->header.frame_id;

  // Convert marker pose to camera <= object transform.
    res->transform_cm = aist_utility::toTransform(*_pose, marker_frame());

  // Lookup world <= effector transform at the moment marker detected.
    try
    {
	res->transform_we = _tf2_buffer.lookupTransform(
				world_frame(), effector_frame(),
				_pose->header.stamp,
				tf2::durationFromSec(1.0));
    }
    catch (const tf2::TransformException& err)
    {
	res->success = false;
	res->message = "failed to look up transform[" + world_frame()
		     + " <= " + effector_frame() + "]: " + err.what();

	RCLCPP_ERROR_STREAM(get_logger(), res->message);
	return;
    }

    res->success = true;

    RCLCPP_INFO_STREAM(get_logger(), "Tcm: " << res->transform_cm.transform);
    RCLCPP_INFO_STREAM(get_logger(), "Twe: " << res->transform_we.transform);
  // RCLCPP_DEBUG_STREAM(get_logger(), "Tcm: " << res->transform_cm.transform);
  // RCLCPP_DEBUG_STREAM(get_logger(), "Twe: " << res->transform_we.transform);

    _Tcm.emplace_back(res->transform_cm);
    _Twe.emplace_back(res->transform_we);
}

void
Calibrator::get_sample_list(const req_p<get_sample_list_t>,
			    const res_p<get_sample_list_t> res)
{
    res->success	= true;
    res->message	= std::to_string(_Tcm.size()) + " samples in hand.";
    res->transform_cm	= _Tcm;
    res->transform_we	= _Twe;

    RCLCPP_INFO_STREAM(get_logger(), "get_sample_list(): " << res->message);
}

void
Calibrator::compute_calibration(const req_p<compute_calibration_t>,
				const res_p<compute_calibration_t> res)
{
    try
    {
	RCLCPP_INFO_STREAM(get_logger(),
			   "compute_calibration(): computing with "
			   << (_use_dual_quaternion ? "DUAL quaternion"
						    : "SINGLE quaternion")
			   << " algorithm...");

	std::vector<TU::Transform<double> >	Tcm, Twe;
	for (size_t i = 0; i < _Tcm.size(); ++i)
	{
	    Tcm.emplace_back(_Tcm[i].transform);
	    Twe.emplace_back(_Twe[i].transform);
	}

	const auto	Tec = (_use_dual_quaternion ?
			       TU::cameraToEffectorDual(Tcm, Twe) :
			       TU::cameraToEffectorSingle(Tcm, Twe));
	const auto	Twm = TU::objectToWorld(Tcm, Twe, Tec);
	const auto	now = get_clock()->now();
	_Tec.header.stamp = now;
	_Tec.transform	  = Tec;
	_Twm.header.stamp = now;
	_Twm.transform	  = Twm;

	const auto	error = TU::evaluateAccuracy(Tcm, Twe, Tec, Twm);
	res->success		    = true;
	res->message		    = "Calibrator::compute_calibration(): succeeded";
	res->transform_ec	    = _Tec;
	res->transform_wm	    = _Twm;
	res->mean_translation_error = error.mean_translation_error;
	res->max_translation_error  = error.max_translation_error;
	res->mean_rotation_error    = error.mean_rotation_error;
	res->max_rotation_error	    = error.max_rotation_error;

	RCLCPP_INFO_STREAM(get_logger(), res->message);
    }
    catch (const std::exception& err)
    {
	res->success = false;
	res->message = "Calibrator::compute_calibration(): "
		     + std::string(err.what());
	RCLCPP_ERROR_STREAM(get_logger(), res->message);
    }
}

void
Calibrator::reset(const req_p<empty_t>, const res_p<empty_t>)
{
    _Tcm.clear();
    _Twe.clear();

    RCLCPP_INFO_STREAM(get_logger(), "reset(): all samples cleared.");
}

}	// namespace aist_handeye_calibration

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_handeye_calibration::Calibrator)
