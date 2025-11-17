/*!
* \file		Detector3D.cpp
* \author	Toshio UESHIBA
* \brief	ARuCo marker detector using both intensity and depth images
*/
#include <iostream>
#include <limits>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <aruco/aruco.h>
#include <aist_utility/opencv.hpp>
#include <aist_utility/sensor_msgs.hpp>
#include <aist_utility/fileio.hpp>

namespace aist_aruco_ros
{
/************************************************************************
*  static functions							*
************************************************************************/
static inline rclcpp::SubscriptionOptions
create_subscription_options(const rclcpp::CallbackGroup::SharedPtr& cbg)
{
    rclcpp::SubscriptionOptions	options;
    options.callback_group = cbg;
    return options;
}

template <class T> inline T
val(const sensor_msgs::msg::Image& image_msg, int u, int v)
{
    using namespace	sensor_msgs;

    if (image_msg.encoding == image_encodings::TYPE_16UC1)
	return T(0.001) * *reinterpret_cast<const uint16_t*>(
				image_msg.data.data() + v*image_msg.step
						      + u*sizeof(uint16_t));
    else
	return *reinterpret_cast<const T*>(image_msg.data.data()
					   + v*image_msg.step + u*sizeof(T));
}

static aruco::CameraParameters
rosCameraInfo2ArucoCamParams(const sensor_msgs::msg::CameraInfo& cinfo,
			     bool useRectifiedParameters)
{
    cv::Mat		cameraMatrix(3, 3, CV_64FC1, 0.0);
    cv::Mat		distorsionCoeff(4, 1, CV_64FC1, 0.0);
    const cv::Size	size(cinfo.width, cinfo.height);

    if (useRectifiedParameters)
    {
	cameraMatrix.at<double>(0, 0) = cinfo.p[0];
	cameraMatrix.at<double>(0, 1) = cinfo.p[1];
	cameraMatrix.at<double>(0, 2) = cinfo.p[2];
	cameraMatrix.at<double>(1, 0) = cinfo.p[4];
	cameraMatrix.at<double>(1, 1) = cinfo.p[5];
	cameraMatrix.at<double>(1, 2) = cinfo.p[6];
	cameraMatrix.at<double>(2, 0) = cinfo.p[8];
	cameraMatrix.at<double>(2, 1) = cinfo.p[9];
	cameraMatrix.at<double>(2, 2) = cinfo.p[10];
    }
    else
    {
	for (size_t i = 0; i < 9; ++i)
	    *(cameraMatrix.ptr<double>() + i) = cinfo.k[i];

	for (size_t i = 0; i < std::min(size_t(4), cinfo.d.size()); ++i)
	    *(distorsionCoeff.ptr<double>() + i) = cinfo.d[i];
    }

    return {cameraMatrix, distorsionCoeff, size};
}

/************************************************************************
*  class Detector3D							*
************************************************************************/
class Detector3D : public rclcpp::Node
{
  private:
    using camera_info_t	= sensor_msgs::msg::CameraInfo;
    using image_t	= sensor_msgs::msg::Image;
    using cloud_t	= sensor_msgs::msg::PointCloud2;
    using depth_sync_t	= message_filters::TimeSynchronizer<image_t, image_t,
							    camera_info_t>;
    using cloud_sync_t	= message_filters::TimeSynchronizer<cloud_t,
							    camera_info_t>;
    using pose_t	= geometry_msgs::msg::PoseStamped;
    using mdetector_t	= aruco::MarkerDetector;
    using mparams_t	= mdetector_t::Params;
    using marker_info_t	= aruco::Marker3DInfo;
    using marker_map_t	= aruco::MarkerMap;
    using point3_t	= cv::Vec<float, 3>;
    using ddr_t		= ddynamic_reconfigure2::DDynamicReconfigure<>;

    template <class MSG>
    using pub_p		= typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using msg_cp	= typename MSG::ConstSharedPtr;

    struct rgb_t	{ uint8_t r, g, b; };

  public:
		Detector3D(const rclcpp::NodeOptions& options)		;

  private:
    void	set_min_marker_size(double size)			;
    void	set_enclosed_marker(bool enable)			;
    void	set_detection_mode(int mode)				;
    void	set_dictionary(const std::string& dict)			;
    void	detect_marker_from_depth_cb(
		    const msg_cp<image_t>&	 image_msg,
		    const msg_cp<image_t>&	 depth,
		    const msg_cp<camera_info_t>& cinfo)			;
    void	detect_marker_from_cloud_cb(
		    const msg_cp<cloud_t>&	 cloud,
		    const msg_cp<camera_info_t>& cinfo)			;
    template <class MSG>
    void	detect_marker(const MSG& msg,
			      const camera_info_t& cinfo,
			      cv::Mat& img)				;
    static void	publish_image(const std_msgs::msg::Header& header,
			      const cv::Mat& img,
			      const image_transport::Publisher& pub)	;
    template <class ITER>
    void	publish_transform(ITER begin, ITER end,
				  const std_msgs::msg::Header& header,
				  const std::string& marker_frame,
				  bool publish_pose)			;
    template <class MSG> std::vector<point3_t>
		get_marker_corners(const aruco::Marker& marker,
				   const MSG& msg, cv::Mat& img) const	;
    template <class T> cv::Vec<T, 3>
		view_vector(T u, T v)				const	;
    template <class T> cv::Vec<T, 3>
		at(const image_t& depth, int u, int v)		const	;
    template <class T> cv::Vec<T, 3>
		at(const cloud_t& cloud, int u, int v)		const	;
    template <class T, class MSG> cv::Vec<T, 3>
		at(const MSG& msg, T u, T v)			const	;

  private:
    ddr_t					_ddr;

  // transformation stuff
    tf2_ros::TransformBroadcaster		_broadcaster;
    std::string					_marker_frame;

  // input camera_info/image stuff
    const rclcpp::SubscriptionOptions		_subscription_options;
    image_transport::ImageTransport		_it;
    image_transport::SubscriberFilter		_image_sub;
    image_transport::SubscriberFilter		_depth_sub;
    message_filters::Subscriber<cloud_t>	_cloud_sub;
    message_filters::Subscriber<camera_info_t>	_cinfo_sub;
    depth_sync_t				_depth_sync;
    cloud_sync_t				_cloud_sync;

  // camera_info stuff
    aruco::CameraParameters			_camParam;
    bool					_useRectifiedImages;
    tf2::Transform				_rightToLeft;

  // output stuff
    const image_transport::Publisher		_result_pub;
    const image_transport::Publisher		_debug_pub;
    const pub_p<pose_t>				_pose_pub;

    mdetector_t					_marker_detector;
    marker_map_t				_marker_map;
    float					_marker_size;
    bool					_useSimilarity;
    double					_planarityTolerance;
};

Detector3D::Detector3D(const rclcpp::NodeOptions& options)
    :rclcpp::Node("detector_3d", options),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _broadcaster(*this),
     _marker_frame(ddynamic_reconfigure2::declare_read_only_parameter(
		       this, "marker_frame", "marker_frame")),
     _subscription_options(
	 create_subscription_options(
	     create_callback_group(
		 rclcpp::CallbackGroupType::MutuallyExclusive))),
     _it(rclcpp::Node::SharedPtr(this)),
     _image_sub(),
     _depth_sub(),
     _cloud_sub(this, "pointcloud",
		rmw_qos_profile_default, _subscription_options),
     _cinfo_sub(this, "camera_info",
		rmw_qos_profile_default, _subscription_options),
     _depth_sync(_image_sub, _depth_sub, _cinfo_sub, 3),
     _cloud_sync(_cloud_sub, _cinfo_sub, 3),
     _camParam(),
     _useRectifiedImages(ddynamic_reconfigure2::declare_read_only_parameter(
			     this, "image_is_rectified", false)),
     _rightToLeft(),
     _result_pub(_it.advertise("~/result", 1)),
     _debug_pub( _it.advertise("~/debug",  1)),
     _pose_pub(create_publisher<pose_t>("~/pose", 1)),
     _marker_detector(),
     _marker_map(),
     _marker_size(ddynamic_reconfigure2::declare_read_only_parameter(
		      this, "marker_size", 0.05)),
     _useSimilarity(false),
     _planarityTolerance(0.001)
{
    using namespace	std::placeholders;

  // Load marker map.
    if (const auto marker_map_file = ddynamic_reconfigure2::
					 declare_read_only_parameter(
					     this, "marker_map_file", "");
	marker_map_file != "")
    {
	try
	{
	    _marker_map.readFromFile(aist_utility::filepath_from_url(
					 marker_map_file));

	    RCLCPP_INFO_STREAM(get_logger(), "Loaded marker map["
			       << marker_map_file << ']');
	}
	catch (const std::exception& err)
	{
	    RCLCPP_ERROR_STREAM(get_logger(), "Failed to load marker map["
				<< marker_map_file << ']');
	    throw;
	}

	_marker_size = cv::norm(_marker_map[0].points[0] -
				_marker_map[0].points[1]);
    }

  // Set minimum marker size and setup ddynamic_reconfigure service for it.
    _ddr.registerVariable<double>("min_marker_size",
				  _marker_detector.getParameters().minSize,
				  std::bind(&Detector3D::set_min_marker_size,
					    this, _1),
				  "Minimum marker size", {0.0, 1.0});

  // Set a parameter for specifying whether the marker is enclosed or not.
    _ddr.registerVariable<bool>("enclosed_marker",
				_marker_detector.getParameters().enclosedMarker,
				std::bind(&Detector3D::set_enclosed_marker,
					  this, _1),
				"Detect enclosed marker");

  // Set detection mode and setup ddynamic_reconfigure service for it.
    _ddr.registerEnumVariable<int>("detection_mode",
				   _marker_detector.getDetectionMode(),
				   std::bind(&Detector3D::set_detection_mode,
					     this, _1),
				   "Marker detection mode",
				   {{"NORMAL",	   aruco::DM_NORMAL},
				    {"FAST",	   aruco::DM_FAST},
				    {"VIDEO_FAST", aruco::DM_VIDEO_FAST}});

  // Set dictionary and setup ddynamic_reconfigure service for it.
    _ddr.registerEnumVariable<std::string>(
	"dictionary", "ARUCO",
	std::bind(&Detector3D::set_dictionary, this, _1),
	"Dictionary",
	{{"ARUCO",		"ARUCO"},
	 {"ARUCO_MIP_25h7",	"ARUCO_MIP_25h7"},
	 {"ARUCO_MIP_16h3",	"ARUCO_MIP_16h3"},
	 {"ARTAG",		"ARTAG"},
	 {"ARTOOLKITPLUS",	"ARTOOLKITPLUS"},
	 {"ARTOOLKITPLUSBCH",	"ARTOOLKITPLUSBCH"},
	 {"TAG16h5",		"TAG16h5"},
	 {"TAG25h7",		"TAG25h7"},
	 {"TAG25h9",		"TAG25h9"},
	 {"TAG36h11",		"TAG36h11"},
	 {"TAG36h10",		"TAG36h10"},
	 {"CUSTOM",		"CUSTOM"}});

  // Set transformation type and setup ddynamic_reconfigure service for it.
    _ddr.registerVariable<bool>(
	"use_similarity", &_useSimilarity,
	"Use similarity transformation to determine marker poses.");

  // Set planarity tolerance and setup ddynamic_recoconfigure service for it.
    _ddr.registerVariable<double>(
    	"planarity_tolerance", &_planarityTolerance,
    	"Planarity tolerance for extracting marker region(in meters)",
    	{0.0005, 0.05});

  // Register callback for marker detection.
    _image_sub.subscribe(this, "image", "raw",
			 rmw_qos_profile_default, _subscription_options);
    _depth_sub.subscribe(this, "depth", "raw",
			 rmw_qos_profile_default, _subscription_options);
    _depth_sync.registerCallback(&Detector3D::detect_marker_from_depth_cb,
				 this);
    _cloud_sync.registerCallback(&Detector3D::detect_marker_from_cloud_cb,
				 this);
}

void
Detector3D::set_min_marker_size(double size)
{
    _marker_detector.setDetectionMode(_marker_detector.getDetectionMode(),
				      size);

    RCLCPP_INFO_STREAM(get_logger(), "Set min_marker_size[" << size << ']');
}

void
Detector3D::set_enclosed_marker(bool enable)
{
    _marker_detector.getParameters().detectEnclosedMarkers(enable);

    RCLCPP_INFO_STREAM(get_logger(), "Set enclosed_marker["
		       << std::boolalpha << enable << ']');
}

void
Detector3D::set_detection_mode(int mode)
{
    _marker_detector.setDetectionMode(aruco::DetectionMode(mode),
				      _marker_detector.getParameters().minSize);

    RCLCPP_INFO_STREAM(get_logger(), "Set detection_mode[" << mode << ']');
}

void
Detector3D::set_dictionary(const std::string& dict)
{
    _marker_detector.setDictionary(dict);
    _marker_map.setDictionary(dict);

    RCLCPP_INFO_STREAM(get_logger(), "Set dictionary[" << dict << ']');
}

void
Detector3D::detect_marker_from_depth_cb(const msg_cp<image_t>& image,
					const msg_cp<image_t>& depth,
					const msg_cp<camera_info_t>& cinfo)
{
    auto	img = cv_bridge::toCvCopy(image,
					  sensor_msgs::image_encodings::RGB8)
		    ->image;
    detect_marker(*depth, *cinfo, img);
}

void
Detector3D::detect_marker_from_cloud_cb(const msg_cp<cloud_t>& cloud,
					const msg_cp<camera_info_t>& cinfo)
{
    if (cloud->is_dense)
    {
	RCLCPP_ERROR_STREAM(get_logger(), "Ordered point cloud required! Cloud with size["
			    << cloud->width << 'x' << cloud->height
			    << "] supplied.");
	return;
    }

    cv::Mat	img(cloud->height, cloud->width, CV_8UC3);
    aist_utility::pointcloud_to_rgb(*cloud, img.ptr<rgb_t>());
    detect_marker(*cloud, *cinfo, img);
}

template <class MSG> void
Detector3D::detect_marker(const MSG& msg, const camera_info_t& cinfo,
			  cv::Mat& img)
{
  // Convert camera_info to aruco camera parameters
    _camParam = rosCameraInfo2ArucoCamParams(cinfo, _useRectifiedImages);

  // Handle cartesian offset between stereo pairs.
  // See the sensor_msgs/CamaraInfo documentation for details.
    _rightToLeft.setIdentity();
    _rightToLeft.setOrigin(tf2::Vector3(-cinfo.p[3] / cinfo.p[0],
					-cinfo.p[7] / cinfo.p[5],
					0.0));

  // Detect markers. Results will go into "markers"
    std::vector<aruco::Marker>	markers;
    _marker_detector.detect(img, markers, _camParam, _marker_size, false);

  // Show also the internal image resulting from the threshold operation
    publish_image(msg.header, _marker_detector.getThresholdedImage(),
		  _debug_pub);

    if (_marker_map.size() > 0)
    {
	std::vector<std::pair<point3_t, point3_t> >	pairs;

	for (const auto& marker : markers)
	{
	  // For each marker, draw info and its boundaries in the image.
	    marker.draw(img, cv::Scalar(0, 0, 255), 2);
	    aruco::CvDrawingUtils::draw3dAxis(
		img, const_cast<aruco::Marker&>(marker), _camParam);

	    const auto	corners = get_marker_corners(marker, msg, img);
	    const auto	i = _marker_map.getIndexOfMarkerId(marker.id);

	    if (i < 0 || corners.size() < 4)
		continue;

	    const auto&	markerinfo = _marker_map[i];
	    for (size_t j = 0; j < corners.size(); ++j)
		pairs.push_back(std::make_pair(markerinfo[j], corners[j]));
	}

	publish_transform(pairs.begin(), pairs.end(),
			  msg.header, _marker_frame, true);
    }
    else	// marker_id not specified.
    {
      // Publish all the detected markers.
	for (const auto& marker : markers)
	{
	  // For each marker, draw info and its boundaries in the image.
	    marker.draw(img, cv::Scalar(0, 0, 255), 2);
	    aruco::CvDrawingUtils::draw3dAxis(
		img, const_cast<aruco::Marker&>(marker), _camParam);

	    const auto	corners = get_marker_corners(marker, msg, img);
	    if (corners.size() < 4)
		continue;

	    std::vector<std::pair<point3_t, point3_t> >	pairs;
	    const auto	half_size = _marker_size/2;
	    pairs.push_back(std::make_pair(point3_t(-half_size, -half_size, 0),
					   corners[0]));
	    pairs.push_back(std::make_pair(point3_t(-half_size,  half_size, 0),
					   corners[1]));
	    pairs.push_back(std::make_pair(point3_t( half_size,  half_size, 0),
					   corners[2]));
	    pairs.push_back(std::make_pair(point3_t( half_size, -half_size, 0),
					   corners[3]));

	    publish_transform(pairs.begin(), pairs.end(), msg.header,
			      _marker_frame + std::to_string(marker.id),
			      false);
	}
    }

    publish_image(msg.header, img, _result_pub);
}

void
Detector3D::publish_image(const std_msgs::msg::Header& header,
			  const cv::Mat& img,
			  const image_transport::Publisher& pub)
{
    using namespace	sensor_msgs;

    if (pub.getNumSubscribers() > 0)
	pub.publish(cv_bridge::CvImage(header, image_encodings::RGB8, img)
		    .toImageMsg());
}

template <class ITER> void
Detector3D::publish_transform(ITER begin, ITER end,
			      const std_msgs::msg::Header& header,
			      const std::string& marker_frame,
			      bool publish_pose)
{
    using element_t	= typename std::iterator_traits<ITER>::value_type
				      ::first_type::value_type;
    using matrix33_t	= cv::Matx<element_t, 3, 3>;
    using vector3_t	= cv::Matx<element_t, 3, 1>;
    using rigidity_t	= aist_utility::opencv::Rigidity<element_t, 3>;
    using similarity_t	= aist_utility::opencv::Similarity<element_t, 3>;

    try
    {
	matrix33_t	R;
	vector3_t	t;

	if (_useSimilarity)
	{
	    similarity_t	similarity;
	    const auto		residual = similarity.fit(begin, end);

	    RCLCPP_DEBUG_STREAM(get_logger(),
				"Fitted similarity transformation: scale = "
				<< similarity.s() << ", residual = "
				<< residual);

	    R = similarity.R();
	    t = similarity.t() * (1/similarity.s());
	}
	else
	{
	    rigidity_t	rigidity;
	    const auto	residual = rigidity.fit(begin, end);

	    RCLCPP_DEBUG_STREAM(get_logger(),
				"Fitted rigid transformation: residual = "
				<< residual);

	    R = rigidity.R();
	    t = rigidity.t();
	}

	tf2::Transform	transform(tf2::Matrix3x3(R(0, 0), R(0, 1), R(0, 2),
						 R(1, 0), R(1, 1), R(1, 2),
						 R(2, 0), R(2, 1), R(2, 2)),
				  tf2::Vector3(t(0), t(1), t(2)));
	geometry_msgs::msg::TransformStamped	transformMsg;
	transformMsg.header	    = header;
	transformMsg.child_frame_id = marker_frame;
	transformMsg.transform	    = tf2::toMsg(_rightToLeft * transform);
	_broadcaster.sendTransform(transformMsg);

	if (publish_pose && _pose_pub->get_subscription_count() > 0)
	{
	    auto	poseMsg = std::make_unique<pose_t>();
	    poseMsg->header = header;
	    tf2::toMsg(transform, poseMsg->pose);
	    _pose_pub->publish(std::move(poseMsg));
	}
    }
    catch (const std::exception& err)
    {
	RCLCPP_WARN_STREAM(get_logger(), err.what());
    }
}

template <class MSG> std::vector<Detector3D::point3_t>
Detector3D::get_marker_corners(const aruco::Marker& marker,
			       const MSG& msg, cv::Mat& image) const
{
    using element_t	= point3_t::value_type;
    using plane_t	= aist_utility::opencv::Plane<element_t, 3>;

    std::vector<point3_t>	corners(0);

    try
    {
      // Compute initial marker plane.
	std::vector<point3_t>	corner_points;
	for (const auto& corner : marker)
	{
	    const auto	point = at<element_t>(msg, corner.x, corner.y);

	    if (point(2) != element_t(0))
		corner_points.push_back(point);
	}

	plane_t	plane(corner_points.cbegin(), corner_points.cend());

      // Compute 2D bounding box of marker.
	const int	u0 = std::floor(std::min({marker[0].x, marker[1].x,
						  marker[2].x, marker[3].x}));
	const int	v0 = std::floor(std::min({marker[0].y, marker[1].y,
						  marker[2].y, marker[3].y}));
	const int	u1 = std::ceil( std::max({marker[0].x, marker[1].x,
						  marker[2].x, marker[3].x}));
	const int	v1 = std::ceil( std::max({marker[0].y, marker[1].y,
						  marker[2].y, marker[3].y}));

      // Select 3D points close to the initial plane within the bounding box.
	std::vector<point3_t>	points;
	for (auto v = v0; v <= v1; ++v)
	    for (auto u = u0; u <= u1; ++u)
	    {
		const auto	point = at<element_t>(msg, u, v);

		if (point(2) != element_t(0) &&
		    plane.distance(point) < _planarityTolerance)
		{
		    points.push_back(point);
		    image.at<rgb_t>(v, u).b = 0;
		}
	    }

      // Fit a plane to seleceted inliers.
	plane.fit(points.cbegin(), points.cend());

      // Compute 3D coordinates of marker corners and then publish.
	for (const auto& corner : marker)
	    corners.push_back(plane.cross_point(view_vector(corner.x,
							    corner.y)));
    }
    catch (const std::exception& err)
    {
	RCLCPP_WARN_STREAM(get_logger(), err.what());
    }

    return corners;
}

template <class T> inline cv::Vec<T, 3>
Detector3D::view_vector(T u, T v) const
{
    cv::Mat_<cv::Point_<T> >	uv(1, 1), xy(1, 1);
    uv(0) = {u, v};
    cv::undistortPoints(uv, xy, _camParam.CameraMatrix, _camParam.Distorsion);

    return {xy(0).x, xy(0).y, T(1)};
}

template <class T> inline cv::Vec<T, 3>
Detector3D::at(const image_t& depth, int u, int v) const
{
    if (u < 0 || u >= int(depth.width) ||
	v < 0 || v >= int(depth.height))
	return {T(0), T(0), T(0)};

    const auto	xyz = view_vector<T>(u, v);
    const auto	d   = val<T>(depth, u, v);

    return {xyz[0]*d, xyz[1]*d, d};
}

template <class T> inline cv::Vec<T, 3>
Detector3D::at(const cloud_t& cloud, int u, int v) const
{
    if (u < 0 || u >= int(cloud.width) ||
	v < 0 || v >= int(cloud.height))
	return {T(0), T(0), T(0)};

    sensor_msgs::PointCloud2ConstIterator<T>	xyz(cloud, "x");
    xyz += (v * cloud.width + u);

    return (!std::isnan(xyz[2]) && xyz[2] > T(0) ?
	    cv::Vec<T, 3>{xyz[0], xyz[1], xyz[2]} :
	    cv::Vec<T, 3>{T(0), T(0), T(0)});
}

template <class T, class MSG> cv::Vec<T, 3>
Detector3D::at(const MSG& msg, T u, T v) const
{
    const int	u0 = std::floor(u);
    const int	v0 = std::floor(v);
    const int	u1 = std::ceil(u);
    const int	v1 = std::ceil(v);
    for (auto vv = v0; vv <= v1; ++vv)
	for (auto uu = u0; uu <= u1; ++uu)
	{
	    const auto	xyz = at<T>(msg, uu, vv);
	    if (xyz[2] != T(0))
		return xyz;
	}

    return {T(0), T(0), T(0)};
}
}	// namespace aist_aruco_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_aruco_ros::Detector3D)
