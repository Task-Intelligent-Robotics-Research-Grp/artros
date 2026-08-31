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
 *  \file	DepthFilter.cpp
 *  \author	Toshio Ueshiba
 *  \brief	ROS node for applying filters to depth images
 */
#include <cstdlib>	// for getenv()
#include <sys/stat.h>	// for mkdir()
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include <aist_utility/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <aist_utility/tiff.hpp>
#include <aist_utility/ply.hpp>
#include <aist_utility/sensor_msgs.hpp>
#include <aist_utility/opencv.hpp>

namespace aist_depth_filter
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> inline bool
is_valid(T val)
{
    return (val != T(0) && !std::isnan(val));
}

inline rclcpp::SubscriptionOptions
create_subscription_options(rclcpp::CallbackGroup::SharedPtr cbg)
{
    rclcpp::SubscriptionOptions	options;
    options.callback_group = cbg;
    return options;
}

/************************************************************************
*  class DepthFilter							*
************************************************************************/
class DepthFilter : public rclcpp::Node
{
  public:
    using value_t	   = float;
    using callback_group_p = rclcpp::CallbackGroup::SharedPtr;
    using camera_info_t	   = sensor_msgs::msg::CameraInfo;
    using image_t	   = sensor_msgs::msg::Image;

  private:
    template <class MSG>
    using msg_cp	= typename MSG::ConstSharedPtr;
    template <class MSG>
    using msg_p		= typename MSG::UniquePtr;
    template <class MSG>
    using pub_p		= typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using sub_p		= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using srv_p		= typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_cp	= typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p		= typename SRV::Response::SharedPtr;

    using sync_t	= message_filters::TimeSynchronizer<
			      camera_info_t, image_t, image_t, image_t>;
    using sync2_t	= message_filters::TimeSynchronizer<
			      camera_info_t, image_t, image_t>;
    using ddr_t		= ddynamic_reconfigure2::DDynamicReconfigure<>;
    using trigger_t	= std_srvs::srv::Trigger;

  public:
		DepthFilter(const rclcpp::NodeOptions& options)		;

  private:
    template <class T>
    void	setVariable(T DepthFilter::* p, T value)		;
    void	save_bg_cb(req_cp<trigger_t>, res_p<trigger_t> res)	;
    void	filter_with_normal_cb(
		    const msg_cp<camera_info_t>& camera_info,
		    const msg_cp<image_t>&	 image,
		    const msg_cp<image_t>&	 depth,
		    const msg_cp<image_t>&	 normal)		;
    void	filter_without_normal_cb(
		    const msg_cp<camera_info_t>& camera_info,
		    const msg_cp<image_t>&	 image,
		    const msg_cp<image_t>&	 depth)			;

    template <class T> msg_p<image_t>
		filter(const camera_info_t& camera_info,
		       const msg_p<image_t>& depth)			;
    template <class T>
    void	remove_bg(const msg_p<image_t>& depth,
			  const image_t& depth_bg)		const	;
    template <class T>
    void	z_clip(const msg_p<image_t>& depth)		const	;
    template <class T>
    void	scale(const msg_p<image_t>& depth)		const	;
    msg_p<camera_info_t>
		create_subcamera_info(
		    const camera_info_t& camera_into)		const	;
    msg_p<image_t>
		create_subimage(const image_t& image)		const	;
    template <class T> msg_p<image_t>
		create_normal(const camera_info_t& camera_info,
			      const image_t& depth)			;
    msg_p<image_t>
		create_colored_normal(const image_t& normal)	const	;
    std::string	open_dir()					const	;

  private:
  // Service
    const callback_group_p			_service_cbg;
    const srv_p<trigger_t>			_save_bg_srv;

  // Subscribers
    const rclcpp::SubscriptionOptions		_subscription_options;
    image_transport::ImageTransport		_it;
    message_filters::Subscriber<camera_info_t>	_camera_info_sub;
    image_transport::SubscriberFilter		_image_sub;
    image_transport::SubscriberFilter		_depth_sub;
    image_transport::SubscriberFilter		_normal_sub;
    sync_t					_sync_with_normal;
    sync2_t					_sync_without_normal;

  // Publishers
    const image_transport::Publisher		_image_pub;
    const image_transport::Publisher		_depth_pub;
    const image_transport::Publisher		_normal_pub;
    const image_transport::Publisher		_colored_normal_pub;
    const pub_p<camera_info_t>			_camera_info_pub;

  // Depth buffers for backgroud removal
    msg_cp<image_t>				_depth_org;
    msg_cp<image_t>				_depth_bg;

  // Parameters
    ddr_t			_ddr;
    mutable std::mutex		_param_mtx;
    double			_thresh_bg;	// thresh for bg removal
    double			_near;		// clip nearer
    double			_far;		// clip farer
    int				_top;		// clip upper
    int				_bottom;	// clip lower
    int				_left;		// clip left
    int				_right;		// clip right
    double			_scale;		// depth scaling
    int				_window_radius;	// kernel for computing normals

  private:
    constexpr static double	FarMax = 4.0;
};

DepthFilter::DepthFilter(const rclcpp::NodeOptions& options)
    :rclcpp::Node("depth_filter", options),
     _service_cbg(create_callback_group(
		      rclcpp::CallbackGroupType::MutuallyExclusive)),
     _save_bg_srv(create_service<trigger_t>(
		     "~/save_bg",
		     std::bind(&DepthFilter::save_bg_cb, this,
			       std::placeholders::_1, std::placeholders::_2),
		     rclcpp::ServicesQoS(), _service_cbg)),
     _subscription_options(
	 create_subscription_options(
	     create_callback_group(
		 rclcpp::CallbackGroupType::MutuallyExclusive))),
     _it(*this),
     _camera_info_sub(*this, "/camera_info",
		      rclcpp::SystemDefaultsQoS(), _subscription_options),
     _image_sub(),
     _depth_sub(),
     _normal_sub(),
     _sync_with_normal(3, _camera_info_sub,
		       _image_sub, _depth_sub, _normal_sub),
     _sync_without_normal(3, _camera_info_sub, _image_sub, _depth_sub),
     _image_pub(	 _it.advertise("~/image",	   1)),
     _depth_pub(	 _it.advertise("~/depth",	   1)),
     _normal_pub(	 _it.advertise("~/normal",	   1)),
     _colored_normal_pub(_it.advertise("~/colored_normal", 1)),
     _camera_info_pub(create_publisher<camera_info_t>("~/camera_info", 1)),
     _depth_org(nullptr),
     _depth_bg(nullptr),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _param_mtx(),
     _thresh_bg(0.0),
     _near(0.0),
     _far(FarMax),
     _top(0),
     _bottom(2048),
     _left(0),
     _right(3072),
     _scale(1.0),
     _window_radius(2)
{
    using	namespace std::placeholders;

  // Setup parameters
    _ddr.registerVariable<double>("thresh_bg", _thresh_bg,
				  std::bind(&DepthFilter::setVariable<double>,
					    this,
					    &DepthFilter::_thresh_bg, _1),
				  "Threshold value for background removal",
				  {0.0, 0.1});
    _ddr.registerVariable<double>("near", _near,
				  std::bind(&DepthFilter::setVariable<double>,
					    this, &DepthFilter::_near, _1),
				  "Nearest depth value", {0.0, 1.0});
    _ddr.registerVariable<double>("far", _far,
				  std::bind(&DepthFilter::setVariable<double>,
					    this, &DepthFilter::_far, _1),
				  "Farest depth value", {0.0, FarMax});
    _ddr.registerVariable<int>("top", _top,
			       std::bind(&DepthFilter::setVariable<int>,
					 this, &DepthFilter::_top, _1),
			       "Top of ROI", {0, 2048});
    _ddr.registerVariable<int>("bottom", _bottom,
			       std::bind(&DepthFilter::setVariable<int>,
					 this, &DepthFilter::_bottom, _1),
			       "Bottom of ROI", {0, 2048});
    _ddr.registerVariable<int>("left", _left,
			       std::bind(&DepthFilter::setVariable<int>,
					 this, &DepthFilter::_left, _1),
			       "Left of ROI", {0, 3072});
    _ddr.registerVariable<int>("right", _right,
			       std::bind(&DepthFilter::setVariable<int>,
					 this, &DepthFilter::_right, _1),
			       "Top of ROI", {0, 3072});
    _ddr.registerVariable<double>("scale", _scale,
			       std::bind(&DepthFilter::setVariable<double>,
					 this, &DepthFilter::_scale, _1),
			       "Scale factor for depth", {0.5, 1.5});

  // Setup subscribers
    _image_sub.subscribe(*this, "/image", "raw",
			 rclcpp::SystemDefaultsQoS(), _subscription_options);
    _depth_sub.subscribe(*this, "/depth", "raw",
			 rclcpp::SystemDefaultsQoS(), _subscription_options);
    if (ddynamic_reconfigure2::declare_read_only_parameter(this,
							   "subscribe_normal",
							   false))
    {
	_normal_sub.subscribe(*this, "/normal", "raw",
			      rclcpp::SystemDefaultsQoS(),
                              _subscription_options);
	_sync_with_normal.registerCallback(&DepthFilter::filter_with_normal_cb,
					   this);
    }
    else
    {
	_ddr.registerVariable("window_radius", &_window_radius,
			      "Window radius for computing normals", {0, 5});

	_sync_without_normal.registerCallback(
	    &DepthFilter::filter_without_normal_cb, this);
    }

    RCLCPP_INFO_STREAM(get_logger(), "initialized");
}

template <class T> void
DepthFilter::setVariable(T DepthFilter::* p, T value)
{
    const std::lock_guard<std::mutex>	lock(_param_mtx);

    this->*p = value;
}

void
DepthFilter::save_bg_cb(req_cp<trigger_t>, res_p<trigger_t> res)
{
    try
    {
	if (!_depth_org)
	    throw std::runtime_error("no original depth image available!");

	aist_utility::saveTiff(*_depth_org, open_dir() + "/bg.tif");

	_depth_bg  = _depth_org;
	_depth_org = nullptr;

	res->success = true;
	res->message = "succeeded";
    }
    catch (const std::exception& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "DepthFilter::save_bg_cb(): " << err.what());

	res->success = false;
	res->message = "failed";
    }

    RCLCPP_INFO_STREAM(get_logger(),
		       "save background image: " << res->message);
}

void
DepthFilter::filter_with_normal_cb(const msg_cp<camera_info_t>& camera_info,
				   const msg_cp<image_t>& image,
				   const msg_cp<image_t>& depth,
				   const msg_cp<image_t>& normal)
{
    {
	const std::lock_guard<std::mutex>	lock(_param_mtx);

	_top    = std::max(0,     std::min(_top,    int(image->height)));
	_bottom = std::max(_top,  std::min(_bottom, int(image->height)));
	_left   = std::max(0,     std::min(_left,   int(image->width)));
	_right  = std::max(_left, std::min(_right,  int(image->width)));

	if (_top == _bottom || _left == _right)
	    return;
    }

    try
    {
      // Keep pointers to original data.
	_depth_org = depth;

      // Create camera_info according to ROI.
	auto	subcamera_info	= create_subcamera_info(*camera_info);
	auto	subdepth	= create_subimage(*depth);
	auto	subnormal	= create_subimage(*normal);

	if (depth->encoding == sensor_msgs::image_encodings::MONO16 ||
	    depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
	    filter<uint16_t>(*subcamera_info, subdepth);
	else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
	    filter<float>(*subcamera_info, subdepth);

	_camera_info_pub->publish(std::move(subcamera_info));
	_image_pub.publish(create_subimage(*image));
	_depth_pub.publish(std::move(subdepth));
	_colored_normal_pub.publish(create_colored_normal(*subnormal));
	_normal_pub.publish(std::move(subnormal));
    }
    catch (const std::exception& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "DepthFilter::filter_with_normal_cb(): "
			    << err.what());
    }
}

void
DepthFilter::filter_without_normal_cb(const msg_cp<camera_info_t>& camera_info,
				      const msg_cp<image_t>& image,
				      const msg_cp<image_t>& depth)
{
    {
	const std::lock_guard<std::mutex>	lock(_param_mtx);

	_top    = std::max(0,     std::min(_top,    int(image->height)));
	_bottom = std::max(_top,  std::min(_bottom, int(image->height)));
	_left   = std::max(0,     std::min(_left,   int(image->width)));
	_right  = std::max(_left, std::min(_right,  int(image->width)));

	if (_top == _bottom || _left == _right)
	    return;
    }

    try
    {
      // Keep pointers to original data.
	_depth_org = depth;

      // Create camera_info according to ROI.
	auto		subcamera_info = create_subcamera_info(*camera_info);
	auto		subdepth       = create_subimage(*depth);
	msg_p<image_t>	subnormal;

	if (depth->encoding == sensor_msgs::image_encodings::MONO16 ||
	    depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
	    subnormal = filter<uint16_t>(*subcamera_info, subdepth);
	else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
	    subnormal = filter<float>(*subcamera_info, subdepth);

	_camera_info_pub->publish(std::move(subcamera_info));
	_image_pub.publish(create_subimage(*image));
	_depth_pub.publish(std::move(subdepth));
	_colored_normal_pub.publish(create_colored_normal(*subnormal));
	_normal_pub.publish(std::move(subnormal));
    }
    catch (const std::exception& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "DepthFilter::filter_without_normal_cb(): "
			    << err.what());
    }
}

template <class T> DepthFilter::msg_p<DepthFilter::image_t>
DepthFilter::filter(const camera_info_t& camera_info,
		    const msg_p<image_t>& depth)
{
    if (_thresh_bg > 0)
    {
	try
	{
	    if (!_depth_bg)
		_depth_bg = aist_utility::loadTiff(open_dir() + "/bg.tif");

	    remove_bg<T>(depth, *_depth_bg);
	}
	catch (const std::exception& err)
	{
	    _depth_bg = nullptr;
	    _thresh_bg = 0;
	}
    }

    z_clip<T>(depth);
    scale<T>(depth);

    return create_normal<T>(camera_info, *depth);
}

template <class T> void
DepthFilter::remove_bg(const msg_p<image_t>& depth,
		       const image_t& depth_bg) const
{
    int		top, left;
    double	thresh_bg;
    {
	const std::lock_guard<std::mutex>	lock(_param_mtx);

	top	  = _top;
	left	  = _left;
	thresh_bg = _thresh_bg;
    }

    for (u_int v = 0; v < depth->height; ++v)
    {
	using namespace	aist_utility;

	auto	p = ptr<T>(*depth, v);
	auto	b = ptr<T>(depth_bg, v + top) + left;
	for (const auto q = p + depth->width; p != q; ++p, ++b)
	    if (*b != 0 && std::abs(meters(*p) - meters(*b)) < thresh_bg)
		*p = 0;
    }
}

template <class T> void
DepthFilter::z_clip(const msg_p<image_t>& depth) const
{
    double	near, far;
    {
	std::lock_guard<std::mutex>	lock(_param_mtx);

	near = _near;
	far  = _far;
    }

    if (near <= 0.0 && far >= FarMax)
	return;

    for (u_int v = 0; v < depth->height; ++v)
    {
	using namespace	aist_utility;

	const auto	p = ptr<T>(*depth, v);
	std::replace_if(p, p + depth->width,
			[near, far](const auto& val)
			{return (meters(val) < near || meters(val) > far);},
			0);
    }
}

template <class T> void
DepthFilter::scale(const msg_p<image_t>& depth) const
{
    double	scale;
    {
	const std::lock_guard<std::mutex>	lock(_param_mtx);

	scale = _scale;
    }

    if (scale == 1.0)
	return;

    for (u_int v = 0; v < depth->height; ++v)
    {
	using namespace	aist_utility;

	const auto	p = ptr<T>(*depth, v);
	std::transform(p, p + depth->width, p,
		       [scale](const auto& val){ return scale * val; });
    }
}

DepthFilter::msg_p<DepthFilter::camera_info_t>
DepthFilter::create_subcamera_info(const camera_info_t& camera_info) const
{
    std::lock_guard<std::mutex>	lock(_param_mtx);

  // Create camera_info according to ROI.
    msg_p<camera_info_t>	cinfo(new camera_info_t(camera_info));
    cinfo->height = _bottom - _top;
    cinfo->width  = _right  - _left;
    cinfo->k[2]	 -= _left;
    cinfo->k[5]  -= _top;
    cinfo->p[2]  -= _left;
    cinfo->p[6]  -= _top;

    return cinfo;
}

DepthFilter::msg_p<DepthFilter::image_t>
DepthFilter::create_subimage(const image_t& image) const
{
    using	namespace sensor_msgs;
    using	iterator_t = decltype(image.data.begin());

    const auto	nbytesPerPixel = image_encodings::bitDepth(image.encoding)/8
			       * image_encodings::numChannels(image.encoding);
    iterator_t		p;
    msg_p<image_t>	subimage(new image_t);
    subimage->header = image.header;
    {
	std::lock_guard<std::mutex>	lock(_param_mtx);

	subimage->height = _bottom - _top;
	subimage->width	 = _right  - _left;

	p = image.data.begin() + _top*image.step + _left*nbytesPerPixel;
    }
    subimage->encoding	   = image.encoding;
    subimage->is_bigendian = image.is_bigendian;
    subimage->step	   = subimage->width*nbytesPerPixel;
    subimage->data.resize(subimage->height * subimage->step);

    for (auto q = subimage->data.begin(); q != subimage->data.end();
	 q += subimage->step)
    {
	std::copy_n(p, subimage->width*nbytesPerPixel, q);
	p += image.step;
    }

    return subimage;
}

template <class T> DepthFilter::msg_p<DepthFilter::image_t>
DepthFilter::create_normal(const camera_info_t& camera_info,
			   const image_t& depth)
{
  // Computation of normals should be done in double-precision
  // in order to avoid truncation error when sliding windows
    using normal_t	= std::array<value_t, 3>;
    using vector3_t	= cv::Vec<double, 3>;	   // double-precision
    using matrix33_t	= cv::Matx<double, 3, 3>;  // double-precision

  // 1: Allocate image for output normals.
    msg_p<image_t>	normal(new image_t);
    normal->header		= depth.header;
    normal->encoding		= sensor_msgs::image_encodings::TYPE_32FC3;
    normal->height		= depth.height;
    normal->width		= depth.width;
    normal->step		= normal->width * sizeof(normal_t);
    normal->is_bigendian	= false;
    normal->data.resize(normal->height * normal->step);
    std::fill(normal->data.begin(), normal->data.end(), 0);

    const auto	window_radius = _window_radius;
    if (window_radius < 1)
	return normal;

  // 2: Compute 3D coordinates.
    cv::Mat_<vector3_t>		xyz(depth.height, depth.width);
    aist_utility::depth_to_points<T>(camera_info, depth, xyz.begin(),
				     aist_utility::milimeters<T>);

  // 3: Compute normals.
    const auto			ws1 = 2 * window_radius;
    cv::Mat_<int>		n(depth.width - ws1, depth.height);
    cv::Mat_<vector3_t>		c(depth.width - ws1, depth.height);
    cv::Mat_<matrix33_t>	M(depth.width - ws1, depth.height);

  // 3.1: Convovle with a box filter in horizontal direction.
    for (int v = 0; v < n.cols; ++v)
    {
	using namespace	aist_utility;

	auto		sum_n = 0;
	vector3_t	sum_c(0, 0, 0);
	auto		sum_M = matrix33_t::zeros();
	for (int u = 0; u < ws1; ++u)
	{
	    const auto&	head = xyz(v, u);

	    if (is_valid(head(2)))
	    {
		++sum_n;
		sum_c += head;
		sum_M += head % head;
	    }
	}

	for (int u = 0; u < n.rows; ++u)
	{
	    const auto&	head = xyz(v, u + ws1);

	    if (is_valid(head(2)))
	    {
		++sum_n;
		sum_c += head;
 		sum_M += head % head;
	    }

	    n(u, v) = sum_n;
	    c(u, v) = sum_c;
	    M(u, v) = sum_M;

	    const auto&	tail = xyz(v, u);

	    if (is_valid(tail(2)))
	    {
		--sum_n;
		sum_c -= tail;
		sum_M -= tail % tail;
	    }
	}
    }

  // 3.2: Convolve with a box filter in vertical direction.
    for (int u = 0; u < n.rows; ++u)
    {
	using namespace	aist_utility;

	auto	sum_n = 0;
	auto	sum_c = vector3_t::zeros();
	auto	sum_M = matrix33_t::zeros();
	for (int v = 0; v < ws1; ++v)
	{
	    sum_n += n(u, v);
	    sum_c += c(u, v);
	    sum_M += M(u, v);
	}

	auto	norm = ptr<normal_t>(*normal, window_radius)
		     + window_radius + u;
	for (int v = ws1; v < n.cols; ++v)
	{
	    sum_n += n(u, v);
	    sum_c += c(u, v);
	    sum_M += M(u, v);

	    if (sum_n > 3)
	    {
		using namespace	opencv;

		const auto	A = sum_n * sum_M - sum_c % sum_c;
		vector3_t	evalues;
		matrix33_t	evectors;
		cv::eigen(A, evalues, evectors);	// Fit a plane.
		auto		normal   = evectors.row(2).t();
		auto		distance = normal.dot(sum_c) / sum_n;
		if (distance > 0)
		{
		    distance *= -1;
		    normal   *= -1;
		}

		// std::cerr << "evalues = ("
		// 	  << evalues(0) << ", "
		// 	  << evalues(1) << ", "
		// 	  << evalues(2) << ")" << std::endl;
		// std::cerr << "dist = " << dist << ", norm = ("
		// 	  << norm(0) << ", "
		// 	  << norm(1) << ", "
		// 	  << norm(2) << ")" << std::endl;

		*norm  = {value_t(normal(0)),
			  value_t(normal(1)), value_t(normal(2))};
	    }

	    const auto	vh = v - ws1;
	    sum_n -= n(u, vh);
	    sum_c -= c(u, vh);
	    sum_M -= M(u, vh);

	    norm  += normal->width;
	}
    }

    return normal;
}

DepthFilter::msg_p<DepthFilter::image_t>
DepthFilter::create_colored_normal(const image_t& normal) const
{
    using normal_t		= std::array<float, 3>;
    using colored_normal_t	= std::array<uint8_t, 3>;

    msg_p<image_t>	colored_normal(new(image_t));
    colored_normal->header	 = normal.header;
    colored_normal->height	 = normal.height;
    colored_normal->width	 = normal.width;
    colored_normal->encoding	 = sensor_msgs::image_encodings::RGB8;
    colored_normal->is_bigendian = normal.is_bigendian;
    colored_normal->step	 = colored_normal->width
				 * sizeof(colored_normal_t);
    colored_normal->data.resize(colored_normal->height * colored_normal->step);

    for (u_int v = 0; v < normal.height; ++v)
    {
	using namespace	aist_utility;

	const auto	p = ptr<normal_t>(normal, v);
	const auto	q = ptr<colored_normal_t>(*colored_normal, v);

	std::transform(p, p + normal.width, q,
		       [](const auto& norm)
		       {
			   return colored_normal_t(
				       {uint8_t(128 + 127*norm[0]),
					uint8_t(128 + 127*norm[1]),
				        uint8_t(128 + 127*norm[2])});
		       });
    }

    return colored_normal;
}

std::string
DepthFilter::open_dir() const
{
    const auto	home = getenv("HOME");
    if (!home)
	throw std::runtime_error("Environment variable[HOME] is not set.");

    const auto	dir_name = home + std::string("/.ros")
				+ get_namespace();
    struct stat	buf;
    if (stat(dir_name.c_str(), &buf) && mkdir(dir_name.c_str(), S_IRWXU))
	throw std::runtime_error("Cannot create " + dir_name + ": "
						  + strerror(errno));

    return dir_name;
}
}	// namespace aist_depth_filter

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_depth_filter::DepthFilter)
