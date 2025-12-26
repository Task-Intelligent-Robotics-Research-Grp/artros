/*!
*  \file	robot_description_provider.cpp
*  \author	Toshio UESHIBA
*  \brief	Bridge software for publishing URDF and TF to NEP
*/
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <urdf/model.h>
#include <std_msgs/msg/string.hpp>
#include <aist_msgs/srv/get_links.hpp>
#include <aist_msgs/msg/link_geometry.hpp>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <aist_utility/fileio.hpp>

namespace aist_visualization
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

/************************************************************************
*  class RobotDescriptionProvider					*
************************************************************************/
class RobotDescriptionProvider : public rclcpp::Node
{
  private:
    template <class MSG>
    using msg_cp	= typename MSG::ConstSharedPtr;
    template <class MSG>
    using sub_p		= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using srv_p		= typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_cp	= typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p		= typename SRV::Response::SharedPtr;

    using callback_group_p	= rclcpp::CallbackGroup::SharedPtr;
    using string_t		= std_msgs::msg::String;
    using get_links_t		= aist_msgs::srv::GetLinks;
    using link_cp		= urdf::LinkConstSharedPtr;
    using material_cp		= urdf::MaterialConstSharedPtr;
    using Link			= aist_msgs::msg::Link;
    using Links			= std::vector<Link>;
    using LinkGeometry		= aist_msgs::msg::LinkGeometry;
    using LinkMaterial		= aist_msgs::msg::Material;

  public:
    RobotDescriptionProvider(const rclcpp::NodeOptions& options)	;

    void	run()							;

  private:
    void	robot_description_cb(const msg_cp<string_t>& desc)	;
    void	get_links_cb(req_cp<get_links_t>,
			     res_p<get_links_t> res)			;
    void	create_links(const link_cp& parent,
			     const link_cp& current, Links& links) const;
    Link	create_link(const link_cp& parent,
			    const link_cp& current)		const	;
    template <class GEOM_CP> LinkGeometry
		create_link_geometry(const GEOM_CP& geometry)	const	;
    LinkMaterial
		create_link_material(const material_cp& material) const	;

  private:
    const callback_group_p		_robot_description_cbg;
    const sub_p<string_t>		_robot_description_sub;
    const srv_p<get_links_t>		_get_links_srv;
    tf2_ros::Buffer			_tf2_buffer;
    const tf2_ros::TransformListener	_tf2_listener;

    link_cp				_root;
};

RobotDescriptionProvider::RobotDescriptionProvider(
    const rclcpp::NodeOptions& options)
    :rclcpp::Node("robot_description_provider", options),
     _robot_description_cbg(create_callback_group(
				rclcpp::CallbackGroupType::MutuallyExclusive)),
     _robot_description_sub(
	 create_subscription<string_t>(
	     "robot_description", rclcpp::QoS(1).transient_local().reliable(),
	     std::bind(&RobotDescriptionProvider::robot_description_cb,
		       this, std::placeholders::_1),
	     create_subscription_options(_robot_description_cbg))),
     _get_links_srv(create_service<get_links_t>(
			"~/get_links",
			std::bind(&RobotDescriptionProvider::get_links_cb,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2))),
     _tf2_buffer(get_clock()),
     _tf2_listener(_tf2_buffer),
     _root()
{
    RCLCPP_INFO_STREAM(get_logger(), "Initialized.");
}

void
RobotDescriptionProvider::robot_description_cb(const msg_cp<string_t>& desc)
{
    RCLCPP_INFO_STREAM(get_logger(), "Received robot_description message");

    urdf::Model	model;
    if (!model.initString(desc->data))
    {
        RCLCPP_ERROR_STREAM(get_logger(),
			    "Failed to parse urdf from robot_description");
        throw;
    }

  // Get all links in the model.
    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);

  // Set root link from root frame name.
    const auto root_frame = declare_parameter<std::string>("root_frame",
							   "world");
    const auto root_link = std::find_if(links.cbegin(), links.cend(),
                                        [&root_frame](const auto& link)
                                        { return link->name == root_frame; });
    if (root_link == links.cend())
    {
        RCLCPP_WARN_STREAM(get_logger(),
			   "Frame \"" << root_frame << "\" not found.");
        _root = model.getRoot();
    }
    else
        _root = *root_link;
    RCLCPP_INFO_STREAM(get_logger(),
		       "Set root frame to \"" << _root->name << "\".");
}

void
RobotDescriptionProvider::get_links_cb(req_cp<get_links_t>,
				       res_p<get_links_t> res)
{
    try
    {
	create_links(_root, _root, res->links);

	RCLCPP_INFO_STREAM(get_logger(),
			   "Responded to a request for link elements");
    }
    catch (const std::exception& err)
    {
	RCLCPP_ERROR_STREAM(get_logger(), err.what());
    }
}

void
RobotDescriptionProvider::create_links(const link_cp& parent,
				       const link_cp& current,
				       Links& links) const
{
    try
    {
	links.push_back(create_link(parent, current));
    }
    catch (const std::exception& err)
    {
	RCLCPP_WARN_STREAM(get_logger(), err.what());
    }

    for (const auto& child : current->child_links)
        create_links(current, child, links);
}

RobotDescriptionProvider::Link
RobotDescriptionProvider::create_link(const link_cp& parent,
				      const link_cp& current) const
{
  // Set transform of this link.
    Link	link;
    link.transform = _tf2_buffer.lookupTransform(parent->name, current->name,
						 tf2::TimePointZero,
						 tf2::durationFromSec(1.0));

  // Set visual and collision primitives of this link.
    link.visual	   = create_link_geometry(current->visual);
    link.collision = create_link_geometry(current->collision);

  // Set material of the visual primitive.
    if (current->visual)
	link.material = create_link_material(current->visual->material);

    return link;
}

template <class GEOM_CP> RobotDescriptionProvider::LinkGeometry
RobotDescriptionProvider::create_link_geometry(const GEOM_CP& geometry) const
{
    using	sp = shape_msgs::msg::SolidPrimitive;

    LinkGeometry	link_geometry;

    if (!geometry || !geometry->geometry)
    {
	link_geometry.primitive.type = 255;	// null primitive
	return link_geometry;
    }

  // Set origin of this primitive
    link_geometry.origin.position.x    = geometry->origin.position.x;
    link_geometry.origin.position.y    = geometry->origin.position.y;
    link_geometry.origin.position.z    = geometry->origin.position.z;
    link_geometry.origin.orientation.x = geometry->origin.rotation.x;
    link_geometry.origin.orientation.y = geometry->origin.rotation.y;
    link_geometry.origin.orientation.z = geometry->origin.rotation.z;
    link_geometry.origin.orientation.w = geometry->origin.rotation.w;

  // Set goemetry of this primitive.
    switch (geometry->geometry->type)
    {
      case urdf::Geometry::BOX:
      {
	const auto&	dim = static_cast<const urdf::Box*>(
				  geometry->geometry.get())->dim;

	link_geometry.primitive.type = sp::BOX;
	link_geometry.primitive.dimensions.resize(3);
	link_geometry.primitive.dimensions[sp::BOX_X] = dim.x;
	link_geometry.primitive.dimensions[sp::BOX_Y] = dim.y;
	link_geometry.primitive.dimensions[sp::BOX_Z] = dim.z;
	break;
      }
      case urdf::Geometry::SPHERE:
      {
	const auto	radius = static_cast<const urdf::Sphere*>(
				     geometry->geometry.get())->radius;

	link_geometry.primitive.type = sp::SPHERE;
	link_geometry.primitive.dimensions.resize(1);
	link_geometry.primitive.dimensions[sp::SPHERE_RADIUS] = radius;
	break;
      }
      case urdf::Geometry::CYLINDER:
      {
	const auto	cylinder = static_cast<const urdf::Cylinder*>(
					geometry->geometry.get());

	link_geometry.primitive.type = sp::CYLINDER;
	link_geometry.primitive.dimensions.resize(2);
	link_geometry.primitive.dimensions[sp::CYLINDER_HEIGHT]
	    = cylinder->length;
	link_geometry.primitive.dimensions[sp::CYLINDER_RADIUS]
	    = cylinder->radius;
	break;
      }
      case urdf::Geometry::MESH:
      {
	const auto	mesh = static_cast<const urdf::Mesh*>(
					geometry->geometry.get());

	link_geometry.primitive.type = 0;
	link_geometry.primitive.dimensions.resize(3);
	link_geometry.primitive.dimensions[0] = mesh->scale.x;
	link_geometry.primitive.dimensions[1] = mesh->scale.y;
	link_geometry.primitive.dimensions[2] = mesh->scale.z;

      // Extract mesh file path from filename specified in URDF.
	const auto	path = aist_utility::filepath_from_url(mesh->filename);
	RCLCPP_DEBUG_STREAM(get_logger(), "create_link: path=" << path);

      // Load mesh data from file.
	std::ifstream	fin(path, std::ios_base::in | std::ios_base::binary);
	if (!fin)
	    throw std::runtime_error("createLink: cannot open mesh file["
				     + path + ']');
	fin.seekg(0, std::ios_base::end);
	const auto	fsize = fin.tellg();
	fin.seekg(0);
	link_geometry.data.resize(fsize);
	fin.read(reinterpret_cast<char*>(link_geometry.data.data()), fsize);
	RCLCPP_DEBUG_STREAM(get_logger(), "create_link: mesh data size="
			    << link_geometry.data.size());

	break;
      }
      default:
	throw std::runtime_error("Unknown geometry type["
				 + std::to_string(geometry->geometry->type)
				 + ']');
    }

    return link_geometry;
}

RobotDescriptionProvider::LinkMaterial
RobotDescriptionProvider::create_link_material(
    const material_cp& material) const
{
    LinkMaterial	link_material;

    if (!material)
	return link_material;

    link_material.name	  = material->name;
    link_material.color.r = material->color.r;
    link_material.color.g = material->color.g;
    link_material.color.b = material->color.b;
    link_material.color.a = material->color.a;

    if (!material->texture_filename.empty())
    {
	const auto	path = aist_utility::filepath_from_url(
				   material->texture_filename);
	const auto	texture = cv::imread(path, cv::IMREAD_COLOR);
	if (texture.data == nullptr)
	    throw std::runtime_error("Failed to load texture[" + path + ']');

	link_material.texture_height = texture.rows;
	link_material.texture_width  = texture.cols;
	link_material.texture_data.resize(link_material.texture_height *
					  link_material.texture_width  *
					  sizeof(cv::Vec3b));
	cv::Mat	proxy(link_material.texture_height,
		      link_material.texture_width,
		      CV_8UC3, link_material.texture_data.data());
	cv::cvtColor(texture, proxy, cv::COLOR_BGR2RGB);
    }
    else
    {
	link_material.texture_height = 0;
	link_material.texture_width  = 0;
    }

    return link_material;
}
}        // namespace aist_visualization

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_visualization::RobotDescriptionProvider)
