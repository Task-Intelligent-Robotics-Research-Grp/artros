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
 *  \file	DepthFilter.h
 *  \author	Toshio Ueshiba
 *  \brief	Thin wraper of Photoneo Localization SDK
 */
#ifndef DEPTHFILTER_H
#define DEPTHFILTER_H

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include <aist_utility/opencv.hpp>

namespace aist_depth_filter
{
/************************************************************************
*  class DepthFilter							*
************************************************************************/
class DepthFilter : public rclcpp::Node
{
  public:
    using value_t	   = float;
    using callback_group_p = rclcpp::CallbackGroup::SharedPtr;
    using camera_info_t	   = sensor_msgs::msg::CameraInfo;
    using camera_info_cp   = camera_info_t::ConstSharedPtr;
    using camera_info_p    = camera_info_t::UniquePtr;
    using image_t	   = sensor_msgs::msg::Image;
    using image_cp	   = image_t::ConstSharedPtr;
    using image_p	   = image_t::UniquePtr;

  private:
    template <class MSG>
    using pub_p		= typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using sub_p		= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using srv_p		= typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_p		= typename SRV::Request::SharedPtr;
    template <class SRV>
    using res_p		= typename SRV::Response::SharedPtr;

    using sync_t	= message_filters::TimeSynchronizer<
				camera_info_t, image_t, image_t, image_t>;
    using sync2_t	= message_filters::TimeSynchronizer<
				camera_info_t, image_t, image_t>;

    using trigger_t	= std_srvs::srv::Trigger;

  public:
		DepthFilter(const rclcpp::NodeOptions& options)		;

  private:
    void	save_bg_cb(const req_p<trigger_t>,
			   const res_p<trigger_t> res)			;
    void	filter_with_normal_cb(const camera_info_cp camera_info,
				      const image_cp image,
				      const image_cp depth,
				      const image_cp normal)		;
    void	filter_without_normal_cb(const camera_info_cp camera_info,
					 const image_cp image,
					 const image_cp depth)		;

    template <class T>
    image_p	filter(const camera_info_t& camera_info,
		       const image_p& depth)				;
    template <class T>
    void	remove_bg(const image_p& depth,
			  const image_t& depth_bg)		const	;
    template <class T>
    void	z_clip(const image_p& depth)			const	;
    template <class T>
    void	scale(const image_p& depth)			const	;
    camera_info_p
		create_subcamera_info(
		    const camera_info_t& camera_into)		const	;
    image_p	create_subimage(const image_t& image)		const	;
    template <class T>
    image_p	create_normal(const camera_info_t& camera_info,
			      const image_t& depth)			;
    image_p	create_colored_normal(const image_t& normal)	const	;
    std::string	open_dir()					const	;

  private:
    const callback_group_p			_service_cbg;
    const srv_p<trigger_t>			_save_bg_srv;

    image_transport::ImageTransport		_it;

    message_filters::Subscriber<camera_info_t>	_camera_info_sub;
    image_transport::SubscriberFilter		_image_sub;
    image_transport::SubscriberFilter		_depth_sub;
    image_transport::SubscriberFilter		_normal_sub;
    sync_t					_sync_with_normal;
    sync2_t					_sync_without_normal;

    const image_transport::Publisher		_image_pub;
    const image_transport::Publisher		_depth_pub;
    const image_transport::Publisher		_normal_pub;
    const image_transport::Publisher		_colored_normal_pub;
    const pub_p<camera_info_t>			_camera_info_pub;

    ddynamic_reconfigure2::DDynamicReconfigure<>	_ddr;

    image_cp					_depth_org;
    image_cp					_depth_bg;

  // Remove background.
    double					_thresh_bg;
    std::string					_file_bg;

  // Clip outside of [_near, _far].
    double					_near;
    double					_far;

  // Mask outside of ROI.
    int						_top;
    int						_bottom;
    int						_left;
    int						_right;

  // Scaling of depth values.
    double					_scale;

  // Save as OrderPly file.
    std::string					_fileOPly;

  // Radius of window for computing normals.
    int						_window_radius;

  private:
    constexpr static double			FarMax = 4.0;
};

}	// namespace aist_photoneo_localization
#endif	// DEPTHFILTER_H
