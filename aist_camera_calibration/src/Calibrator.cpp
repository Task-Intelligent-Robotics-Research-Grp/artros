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
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <aist_msgs/srv/camera_calibration_take_sample.hpp>
#include <aist_msgs/srv/camera_calibration_get_sample_list.hpp>
#include <aist_msgs/srv/camera_calibration_compute_calibration.hpp>
#include <aist_utility/geometry_msgs.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include "CameraCalibrator.h"
#include "TU/Camera++.h"
#include "TU/Quaternion.h"

namespace aist_camera_calibration
{
/************************************************************************
*  static functions							*
************************************************************************/
  /*
template <class T> std::ostream&
operator <<(std::ostream& out, const TU::Point2<T>& p)
{
    return out << p[0] << ' ' << p[1];
}

template <class T> std::ostream&
operator <<(std::ostream& out, const TU::Point3<T>& p)
{
    return out << p[0] << ' ' << p[1] << ' ' << p[2];
}
  */
template <class S, class T> std::ostream&
operator <<(std::ostream& out, const std::pair<S, T>& p)
{
    return out << p.first << ' ' << p.second;
}

template <class T> std::ostream&
operator <<(std::ostream& out, const std::vector<T>& v)
{
    for (const auto& item : v)
	out << ' ' << item;
    if (!std::is_arithmetic_v<T>)
    	out << std::endl;
    return out;
}

/************************************************************************
*  class Calibrator							*
************************************************************************/
class Calibrator : public rclcpp::Node
{
  public:
    using camera_info_t		= sensor_msgs::msg::CameraInfo;
    using pose_t		= geometry_msgs::msg::PoseStamped;

  private:
    template <class MSG>
    using sub_p		= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using srv_p		= typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_p		= typename SRV::Request::SharedPtr;
    template <class SRV>
    using res_p		= typename SRV::Response::SharedPtr;

    using callback_group_p	= rclcpp::CallbackGroup::SharedPtr;
    using element_t		= double;
    using correses_msg_t	= aist_msgs::msg::PointCorrespondenceArray;
    using correses_set_msg_t	= aist_msgs::msg
					   ::PointCorrespondenceArrayArray;
    using correses_set_msg_cp	= correses_set_msg_t::ConstSharedPtr;
    using empty_t		= std_srvs::srv::Empty;
    using take_sample_t		= aist_msgs::srv::CameraCalibrationTakeSample;
    using get_sample_list_t	= aist_msgs::srv
					   ::CameraCalibrationGetSampleList;
    using compute_calibration_t	= aist_msgs::srv
				      ::CameraCalibrationComputeCalibration;

    using point2_t		= TU::Point2<element_t>;
    using point3_t		= TU::Point3<element_t>;
    using corres22_t		= std::pair<point2_t, point2_t>;
    using corres32_t		= std::pair<point3_t, point2_t>;
    template <class CORRES>
    using correses_t		= std::vector<CORRES>;
    template <class CORRES>
    using correses_set_t	= std::vector<correses_t<CORRES> >;
    template <class CORRES>
    using correses_sets_t	= std::vector<correses_set_t<CORRES> >;
    using camera_t		= TU::Camera<TU::IntrinsicWithDistortion<
						 TU::Intrinsic<element_t> > >;

    template <class SRC_PNT, class DST_PNT=point2_t>
    struct to_corres
    {
	std::pair<SRC_PNT, DST_PNT>
	operator ()(const aist_msgs::msg::PointCorrespondence& corres) const
	{
	    SRC_PNT	p;
	    p[0] = corres.source_point.x;
	    p[1] = corres.source_point.y;
	    DST_PNT	q;
	    q[0] = corres.image_point.x;
	    q[1] = corres.image_point.y;

	    return {p, q};
	}
    };
    template <class DST_PNT>
    struct to_corres<point3_t, DST_PNT>
    {
	std::pair<point3_t, DST_PNT>
	operator ()(const aist_msgs::msg::PointCorrespondence& corres) const
	{
	    point3_t	p;
	    p[0] = corres.source_point.x;
	    p[1] = corres.source_point.y;
	    p[2] = corres.source_point.z;
	    DST_PNT	q;
	    q[0] = corres.image_point.x;
	    q[1] = corres.image_point.y;

	    return {p, q};
	}
    };

    struct to_camera_name
    {
	const std::string&
	operator ()(const correses_msg_t& correses)		const	;
    };

    struct to_camera_info
    {
	to_camera_info(const rclcpp::Time& stamp) :_stamp(stamp)	{}

	camera_info_t
	operator ()(const camera_t& camera,
		    const correses_msg_t& correses)		const	;

      private:
	const rclcpp::Time	_stamp;
    };

    struct to_pose
    {
	pose_t
	operator ()(const camera_t& camera,
		    const correses_msg_t& correses)		const	;
    };

  public:
		Calibrator(const rclcpp::NodeOptions& options)		;

  private:
    void	corres_cb(const correses_set_msg_cp correses_set_msg)	;
    void	take_sample(const req_p<take_sample_t>,
			    const res_p<take_sample_t> res)		;
    void	get_sample_list(const req_p<get_sample_list_t>,
				const res_p<get_sample_list_t> res)	;
    void	compute_calibration(const req_p<compute_calibration_t>,
				    const res_p<compute_calibration_t> res);
    void	reset(const req_p<empty_t>, const res_p<empty_t>)	;

    correses_sets_t<corres22_t>
		convert_correspondences_sets()			const	;
    correses_set_t<corres32_t>
		rearrange_correspondences_sets()		const	;

  private:
    const sub_p<correses_set_msg_t>	_corres_sub;

    const callback_group_p		_take_sample_cbg;
    const srv_p<take_sample_t>		_take_sample_srv;
    const srv_p<get_sample_list_t>	_get_sample_list_srv;
    const srv_p<compute_calibration_t>	_compute_calibration_srv;
    const srv_p<empty_t>		_reset_srv;

    correses_set_msg_cp			_correspondences_set;
    std::mutex				_correspondences_set_mtx;
    std::condition_variable		_correspondences_set_cv;
    const std::chrono::duration<double>	_correspondences_set_timeout;

    std::vector<correses_set_msg_t>	_correspondences_sets;
    std::vector<camera_info_t>		_intrinsics;
    std::vector<pose_t>			_camera_poses;
};

Calibrator::Calibrator(const rclcpp::NodeOptions& options)
    :rclcpp::Node("calibrator", options),
     _corres_sub(create_subscription<correses_set_msg_t>(
		     "point_correspondences_set", 1,
		     std::bind(&Calibrator::corres_cb,
			       this, std::placeholders::_1))),
     _take_sample_cbg(create_callback_group(
			  rclcpp::CallbackGroupType::MutuallyExclusive)),
     _take_sample_srv(create_service<take_sample_t>(
			  "~/take_sample",
			  std::bind(&Calibrator::take_sample, this,
				    std::placeholders::_1,
				    std::placeholders::_2),
			  rclcpp::ServicesQoS(), _take_sample_cbg)),
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
     _correspondences_set(),
     _correspondences_set_mtx(),
     _correspondences_set_cv(),
     _correspondences_set_timeout(
	 ddynamic_reconfigure2::declare_read_only_parameter(
	     this, "correspondences_timeout", 5.0)),
     _correspondences_sets(),
     _intrinsics(),
     _camera_poses()
{
    RCLCPP_INFO_STREAM(get_logger(), "calibrator initialized");
}

void
Calibrator::corres_cb(const correses_set_msg_cp correses_set_msg)
{
  // Check input correspondences set.
    try
    {
	if (correses_set_msg->correspondences_set.empty())
	    throw std::runtime_error("No cameras found in the input correspondences set");
	for (const auto& correspondences
		 : correses_set_msg->correspondences_set)
	    if (correspondences.correspondences.empty())
		throw std::runtime_error("No correspondences found for "
					 + correspondences.camera_name);
    }
    catch (std::runtime_error& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(), "Illegal input correspondences["
			    << err.what() << ']');
	return;
    }

    const std::lock_guard<std::mutex>	lock(_correspondences_set_mtx);
    _correspondences_set = correses_set_msg;
    _correspondences_set_cv.notify_all();
}

void
Calibrator::take_sample(const req_p<take_sample_t>,
			const res_p<take_sample_t> res)
{
    RCLCPP_INFO_STREAM(get_logger(),
		       "new request for CameraCalibrationTakeSample received");

    {
	std::unique_lock<std::mutex>	lock(_correspondences_set_mtx);

	_correspondences_set = nullptr;

      // Get the latest pose of the marker.
	if (!_correspondences_set_cv.wait_for(
		lock, _correspondences_set_timeout,
		[this]{ return _correspondences_set != nullptr; }))
	{
	    res->success = false;
	    res->message = "timeout["
			 + std::to_string(_correspondences_set_timeout.count())
			 + "sec] has expired before correspondences available";

	    RCLCPP_ERROR_STREAM(get_logger(), res->message);
	    return;
	}

	_correspondences_sets.emplace_back(*_correspondences_set);
    }

  // If using planar calibration object, set the reference frame of each
  // camera to the first camera frame.
    auto&	correspondences_set = _correspondences_sets.back();
    const auto	first_camera_frame = correspondences_set.correspondences_set
				    .front().header.frame_id;
    for (auto&& correspondences : correspondences_set.correspondences_set)
	if (correspondences.reference_frame.empty())
	    correspondences.reference_frame = first_camera_frame;

    res->success = true;
    res->message = "take_sample: corresnpondences between "
		 + std::to_string(correspondences_set.
				  correspondences_set.size())
		 + " views obtained";
    res->correspondences_set = correspondences_set.correspondences_set;

    RCLCPP_INFO_STREAM(get_logger(), res->message);
}

void
Calibrator::get_sample_list(const req_p<get_sample_list_t>,
			    const res_p<get_sample_list_t> res)
{
    res->correspondences_sets = _correspondences_sets;
    res->message = "get_sample_list: "
		 + std::to_string(res->correspondences_sets.size())
		 + " samples obtained";

    RCLCPP_INFO_STREAM(get_logger(), res->message);
}

void
Calibrator::compute_calibration(const req_p<compute_calibration_t>,
				const res_p<compute_calibration_t> res)
{
    try
    {
	if (_correspondences_sets.empty())
	    throw std::runtime_error("No correspondence data available!");

      // Take the first sample of correspondences set.
	const auto& first_correspondences_set = _correspondences_sets.front()
						.correspondences_set;

	TU::CameraCalibrator<element_t>	calibrator;
	TU::Array<camera_t>		cameras;

      // If reference and camera frames are identical for the first camera,
      // the calibration object is planar.
	if (const auto& correspondences = first_correspondences_set.front();
	    correspondences.reference_frame == correspondences.header.frame_id)
	{
	    const auto	correses_sets = convert_correspondences_sets();
	    const auto	planes = calibrator.planeCalib(correses_sets.cbegin(),
	    					       correses_sets.cend(),
	    					       cameras, false, true);
	}
	else
	{
	    const auto	correses_set = rearrange_correspondences_sets();

	    cameras.resize(correses_set.size());

	    auto	camera = cameras.begin();
	    for (const auto& correses : correses_set)
	    	calibrator.volumeCalib(correses.cbegin(), correses.cend(),
	    			       *camera++, true);
	}

	res->camera_names.clear();
	res->intrinsics.clear();
	res->camera_poses.clear();

	std::transform(first_correspondences_set.cbegin(),
		       first_correspondences_set.cend(),
		       std::back_inserter(res->camera_names),
		       to_camera_name());
	std::transform(cameras.cbegin(), cameras.cend(),
		       first_correspondences_set.cbegin(),
		       std::back_inserter(res->intrinsics),
		       to_camera_info(get_clock()->now()));
	std::transform(cameras.cbegin(), cameras.cend(),
		       first_correspondences_set.cbegin(),
		       std::back_inserter(res->camera_poses), to_pose());

	res->error   = calibrator.reprojectionError();
	res->success = true;
	res->message = "Calibrator::cmpute_calibration: succesfully computed calibration with reprojection error["
		     + std::to_string(res->error) + "(pix)]";

	_intrinsics   = res->intrinsics;
	_camera_poses = res->camera_poses;

	RCLCPP_INFO_STREAM(get_logger(), res->message);
    }
    catch (const std::exception& err)
    {
	res->success = false;
	res->message = "Calibrator::cmpute_calibration: "
		     + std::string(err.what());
	RCLCPP_ERROR_STREAM(get_logger(), res->message);
    }
}

void
Calibrator::reset(const req_p<empty_t>, const res_p<empty_t>)
{
    _correspondences_sets.clear();
    _intrinsics.clear();
    _camera_poses.clear();

    RCLCPP_INFO_STREAM(get_logger(), "reset(): all samples cleared.");
}

Calibrator::correses_sets_t<Calibrator::corres22_t>
Calibrator::convert_correspondences_sets() const
{
    correses_sets_t<corres22_t>	correses_sets(_correspondences_sets.size());

  // For each marker position...
    auto	correses_set = correses_sets.begin();
    for (const auto& correspondences_set : _correspondences_sets)
    {
      // Allocate #cameras arrays for storing correspondences.
	correses_set->resize(correspondences_set.correspondences_set.size());

      // For each camera...
	auto	correses = correses_set->begin();
	for (const auto& correspondences :
		 correspondences_set.correspondences_set)
	{
	    std::transform(correspondences.correspondences.cbegin(),
			   correspondences.correspondences.cend(),
			   std::back_inserter(*correses++),
			   to_corres<corres22_t::first_type>());
	}

	++correses_set;
    }

    return correses_sets;
}

Calibrator::correses_set_t<Calibrator::corres32_t>
Calibrator::rearrange_correspondences_sets() const
{
    correses_set_t<corres32_t>	correses_set(_correspondences_sets.empty() ?
					     0 :
					     _correspondences_sets.front()
					     .correspondences_set.size());

  // For each marker position...
    for (const auto& correspondences_set : _correspondences_sets)
    {
      // For each camera...
	auto	correses = correses_set.begin();
	for (const auto& correspondences :
		 correspondences_set.correspondences_set)
	{
	    std::transform(correspondences.correspondences.cbegin(),
			   correspondences.correspondences.cend(),
			   std::back_inserter(*correses++),
			   to_corres<corres32_t::first_type>());
	}
    }

    return correses_set;
}

/************************************************************************
*  struct Calibrator::to_camera_name					*
************************************************************************/
const std::string&
Calibrator::to_camera_name::operator()(const correses_msg_t& correses) const
{
    return correses.camera_name;
}

/************************************************************************
*  struct Calibrator::to_camera_info					*
************************************************************************/
Calibrator::camera_info_t
Calibrator::to_camera_info::operator()(const camera_t& camera,
				       const correses_msg_t& correses) const
{
    camera_info_t	camera_info;

    camera_info.header.frame_id = correses.header.frame_id;
    camera_info.header.stamp    = _stamp;

    camera_info.height = correses.height;
    camera_info.width  = correses.width;

  // Set distortion parameters.
    camera_info.distortion_model = "plumb_bob";
    camera_info.d.resize(5);
    camera_info.d[0] = camera.d1();
    camera_info.d[1] = camera.d2();
    camera_info.d[2] = 0.0;
    camera_info.d[3] = 0.0;
    camera_info.d[4] = 0.0;

  // Set intrinsic parameters.
    const auto	K = camera.K();
    camera_info.k[0] = K[0][0];
    camera_info.k[1] = 0.0;
    camera_info.k[2] = K[0][2];
    camera_info.k[3] = 0.0;
    camera_info.k[4] = K[1][1];
    camera_info.k[5] = K[1][2];
    camera_info.k[6] = 0.0;
    camera_info.k[7] = 0.0;
    camera_info.k[8] = 1.0;

  // Set rotation matrix.
    camera_info.r[0] = 1.0;
    camera_info.r[1] = 0.0;
    camera_info.r[2] = 0.0;
    camera_info.r[3] = 0.0;
    camera_info.r[4] = 1.0;
    camera_info.r[5] = 0.0;
    camera_info.r[6] = 0.0;
    camera_info.r[7] = 0.0;
    camera_info.r[8] = 1.0;

  // Set projection matrix.
    camera_info.p[ 0] = camera_info.k[0];
    camera_info.p[ 1] = camera_info.k[1];
    camera_info.p[ 2] = camera_info.k[2];
    camera_info.p[ 3] = 0.0;
    camera_info.p[ 4] = camera_info.k[3];
    camera_info.p[ 5] = camera_info.k[4];
    camera_info.p[ 6] = camera_info.k[5];
    camera_info.p[ 7] = 0.0;
    camera_info.p[ 8] = camera_info.k[6];
    camera_info.p[ 9] = camera_info.k[7];
    camera_info.p[10] = camera_info.k[8];
    camera_info.p[11] = 0.0;

  // Set binning.
    camera_info.binning_x = 0;
    camera_info.binning_y = 0;

    return camera_info;
}

/************************************************************************
*  struct Calibrator::to_pose						*
************************************************************************/
Calibrator::pose_t
Calibrator::to_pose::operator()(const camera_t& camera,
				const correses_msg_t& correses) const
{
    pose_t	pose;
    pose.header.frame_id = correses.reference_frame;
    pose.pose.position.x = camera.t()[0];
    pose.pose.position.y = camera.t()[1];
    pose.pose.position.z = camera.t()[2];

    const TU::Matrix<element_t, 3, 3>	R = transpose(camera.Rt());
    const TU::Quaternion<element_t>	q(R);
    pose.pose.orientation.x = q.vector()[0];
    pose.pose.orientation.y = q.vector()[1];
    pose.pose.orientation.z = q.vector()[2];
    pose.pose.orientation.w = q.scalar();

    return pose;
}
}	// namespace aist_camera_calibration

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_camera_calibration::Calibrator)
