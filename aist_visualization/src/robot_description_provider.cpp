/*!
*  \file	robot_description_provider.cpp
*  \author	Toshio UESHIBA
*  \brief	Bridge software for publishing URDF and TF to NEP
*/
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <urdf_parser/urdf_parser.h>
#include <aist_msgs/GetLinks.h>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace nep_bridge
{
/************************************************************************
*  class RobotDescriptionProvider					*
************************************************************************/
class RobotDescriptionProvider
{
  private:
    using link_cp	= urdf::LinkConstSharedPtr;
    using material_cp   = urdf::MaterialConstSharedPtr;
    using Link          = aist_msgs::Link;
    using Links         = std::vector<Link>;
    using LinkGeometry  = aist_msgs::LinkGeometry;
    using LinkMaterial  = aist_msgs::Material;
    using get_links_req = aist_msgs::GetLinks::Request;
    using get_links_res = aist_msgs::GetLinks::Response;

  public:
		RobotDescriptionProvider(ros::NodeHandle& nh)		;

    void	run()							;

  private:
    bool	get_links_cb(get_links_req&  req,
			     get_links_res& res)			;
    void	create_links(const link_cp& parent,
			     const link_cp& current, Links& links) const;
    Link	create_link(const link_cp& parent,
			    const link_cp& current)		const	;
    template <class GEOM_CP> LinkGeometry
                create_link_geometry(const GEOM_CP& geometry)   const   ;
    LinkMaterial
                create_link_material(const material_cp& material) const ;

    static
    std::string	get_file_path(const std::string& filename)		;

  private:
    tf2_ros::Buffer			_tf2_buffer;
    const tf2_ros::TransformListener	_tf2_listener;
    const ros::ServiceServer		_get_links_srv;

    urdf::ModelInterfaceSharedPtr	_model;
    link_cp				_root;
};

RobotDescriptionProvider::RobotDescriptionProvider(ros::NodeHandle& nh)
    :_tf2_buffer(),
     _tf2_listener(_tf2_buffer),
     _get_links_srv(nh.advertiseService(
			"get_links",
			&RobotDescriptionProvider::get_links_cb, this)),
     _model(),
     _root()
{
  // Load robot model described in URDF.
    const auto	description_param = nh.param<std::string>("description_param",
							  "robot_description");
    std::string	description_xml;
    if (!nh.getParam(description_param, description_xml))
    {
        ROS_ERROR_STREAM("(RobotDescriptionProvider) Failed to get description parameter["
                         << description_param << ']');
        throw;
    }

    _model = urdf::parseURDF(description_xml);
    if (!_model)
    {
        ROS_ERROR_STREAM("(RobotDescriptionProvider) Failed to load urdf from parameter["
                         << description_param << ']');
        throw;
    }

  // Get all links in the model.
    std::vector<urdf::LinkSharedPtr> links;
    _model->getLinks(links);

  // Set root link from root frame name.
    const auto root_frame = nh.param<std::string>("root_frame", "world");
    const auto root_link = std::find_if(links.cbegin(), links.cend(),
                                        [&root_frame](const auto& link)
                                        { return link->name == root_frame; });
    if (root_link == links.cend())
    {
        ROS_WARN_STREAM("(RobotDescriptionProvider) Frame \""
                        << root_frame << "\" not found.");
        _root = _model->getRoot();
    }
    else
        _root = *root_link;
    ROS_INFO_STREAM("(RobotDescriptionProvider) Set root frame to \""
		    << _root->name << "\".");

    ROS_INFO_STREAM("(RobotDescriptionProvider) Initialized.");
}

void
RobotDescriptionProvider::run()
{
    ros::spin();
}

bool
RobotDescriptionProvider::get_links_cb(get_links_req& req,
				       get_links_res& res)
{
    try
    {
	create_links(_root, _root, res.links);

	ROS_INFO_STREAM("(RobotDescriptionProvider) Responded to a request for link elements");
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(RobotDescriptionProvider) " << err.what());
    }

    return true;
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
	ROS_WARN_STREAM("(RobotDescriptionProvider) " << err.what());
    }

    for (const auto& child : current->child_links)
        create_links(current, child, links);
}

RobotDescriptionProvider::Link
RobotDescriptionProvider::create_link(const link_cp& parent,
				      const link_cp& current) const
{
  // Set link transform of this primitive.
    Link	link;
    link.transform = _tf2_buffer.lookupTransform(parent->name,
						 current->name, ros::Time(0));

  // Set visual, material and collision primitives of this link.
    for (const auto& visual : current->visual_array)
    {
        link.visual_array.push_back(create_link_geometry(visual));
        link.material_array.push_back(create_link_material(visual->material));
    }
    for (const auto& collision : current->collision_array)
        link.collision_array.push_back(create_link_geometry(collision));

    return link;
}

template <class GEOM_CP> RobotDescriptionProvider::LinkGeometry
RobotDescriptionProvider::create_link_geometry(const GEOM_CP& geometry) const
{
    using	sp = shape_msgs::SolidPrimitive;

    LinkGeometry        link_geometry;

  // If no geometry is available, return a link with null primitive.
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
	const auto	path = get_file_path(mesh->filename);
	ROS_DEBUG_STREAM("create_link: path=" << path);

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
	ROS_DEBUG_STREAM("create_link: mesh data size="
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
    LinkMaterial        link_material;

    if (!material)
        return link_material;

    link_material.name	 = material->name;
    link_material.color.r = material->color.r;
    link_material.color.g = material->color.g;
    link_material.color.b = material->color.b;
    link_material.color.a = material->color.a;

  // Set material of the primitive.
    if (!material->texture_filename.empty())
    {
	const auto	path = get_file_path(material->texture_filename);
	if (!path.empty())
	{
	    const auto	texture = cv::imread(path, cv::IMREAD_COLOR);
	    if (texture.data == nullptr)
		throw std::runtime_error("Failed to load texture["
					 + path + ']');

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
    }

    return link_material;
}

std::string
RobotDescriptionProvider::get_file_path(const std::string& filename)
{
    constexpr const char*	types[] = {"package://", "file://"};
    std::string			path;
    for (const auto type : types)
    {
	auto	pos1 = filename.find(type, 0);
	if (pos1 == std::string::npos)
	    continue;

	pos1 += strlen(type);

	if (type == types[0])
	{
	    const auto	pos2 = filename.find("/", pos1);
	    const auto	package_name = filename.substr(pos1, pos2 - pos1);
	    const auto	file_name = filename.substr(pos2 + 1);
	    ROS_DEBUG_STREAM("get_file_path: package=" << package_name
			     << " file=" << file_name);
	    path = ros::package::getPath(package_name) + '/' + file_name;
	}
	else
	    path = filename.substr(pos1);
	break;
    }

    return path;
}

}        // namespace nep_bridge

/************************************************************************
*  main function                                                        *
************************************************************************/
int
main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_description_provider");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    //                                ros::console::levels::Debug);

    try
    {
	ros::NodeHandle				nh("~");
        nep_bridge::RobotDescriptionProvider	provider(nh);
        provider.run();
    }
    catch (const std::exception& err)
    {
        std::cerr << err.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        return 1;
    }

    return 0;
}
