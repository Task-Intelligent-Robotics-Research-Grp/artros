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
  \file		textured_mesh_display.cpp
  \author	Toshio Ueshiba
*/
#include <rviz_common/display_context.hpp>
#include <Ogre.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "textured_mesh_display.hpp"

namespace aist_visualization
{
/************************************************************************
*  static functions							*
************************************************************************/
static Ogre::Vector3
fromMsg(const geometry_msgs::msg::Point& p)
{
    return {float(p.x), float(p.y), float(p.z)};
}

/************************************************************************
*  class TexturedMeshDisplay						*
************************************************************************/
/*
 *  public member functions
 */
TexturedMeshDisplay::TexturedMeshDisplay()
    :rviz_common::Display(),
     image_topic_property_(new topic_prop_t("Image Topic", "",
					    "sensor_msgs/msg/Image",
					    "Image topic to subscribe to.",
					    this, SLOT(updateTopic()))),
     mesh_topic_property_(new topic_prop_t("Mesh Topic", "",
					   "aist_msgs/msg/TexturedMeshStamped",
					   "Mesh topic to subscribe to.",
					   this, SLOT(updateTopic()))),
     it_(),
     image_sub_(),
     mesh_sub_(),

     cur_image_(),
     cur_mesh_(),
     image_mtx_(),
     mesh_mtx_(),

     texture_(new texture_t()),
     mesh_node_(),
     manual_object_(),
     mesh_material_()
{
}

TexturedMeshDisplay::~TexturedMeshDisplay()
{
    if (initialized())
	unsubscribe();
}

/*
 *  public mamber functions : Overrides from Display
 */
void
TexturedMeshDisplay::onInitialize()
{
    Display::onInitialize();

    const auto	ros_node = context_->getRosNodeAbstraction().lock();
    image_topic_property_->initialize(ros_node);
    mesh_topic_property_ ->initialize(ros_node);
    it_.reset(new image_transport::ImageTransport(*ros_node->get_raw_node()));
}

void
TexturedMeshDisplay::update(float wall_dt, float ros_dt)
{
    try
    {
	if (cur_image_ && cur_mesh_)
	{
	    createTexture();		// Create texture_ from cur_image_
	    createMesh();		// Create mesh_node_ from cur_mesh_
	    updateMeshProperties();

	    if (!image_topic_property_->getTopicStd().empty() &&
		!mesh_topic_property_->getTopicStd().empty())
	    {
		texture_->update();
		updateCamera();
	    }
	}

	setStatus(rviz_common::properties::StatusProperty::Ok,
		  "Display Image", "ok");
    }
    catch (const std::exception& e)
    {
	setStatus(rviz_common::properties::StatusProperty::Error,
		  "Display Image", e.what());
    }
}

void
TexturedMeshDisplay::reset()
{
    Display::reset();
    texture_->clear();
    context_->queueRender();
    setStatus(rviz_common::properties::StatusProperty::Warn,
	      "Image", "No Image received");
}

/*
 *  protected member functions
 */
void
TexturedMeshDisplay::onEnable()
{
    subscribe();
}

void
TexturedMeshDisplay::onDisable()
{
    unsubscribe();
}

void
TexturedMeshDisplay::fixedFrameChanged()
{
    Display::reset();
}

void
TexturedMeshDisplay::subscribe()
{
    if (!isEnabled())
	return;

    try
    {
	if (!image_topic_property_->getTopicStd().empty() &&
	    !mesh_topic_property_ ->getTopicStd().empty())
	{
	    const auto	ros_node = context_->getRosNodeAbstraction().lock();

	    image_sub_ = it_->subscribe(image_topic_property_->getTopicStd(),
					4,
					&TexturedMeshDisplay::imageCB, this);
	    mesh_sub_ = ros_node->get_raw_node()->create_subscription<mesh_t>(
			    mesh_topic_property_->getTopicStd(), 4,
			    std::bind(&TexturedMeshDisplay::meshCB, this,
				      std::placeholders::_1));

	    setStatus(rviz_common::properties::StatusProperty::Ok,
		      "Image and Mesh topics", "ok");
	}
    }
    catch (const std::exception& e)
    {
	setStatus(rviz_common::properties::StatusProperty::Error,
		  "Image and Mesh topics",
		  QString("Error subscribing: ") + e.what());
    }
}

void
TexturedMeshDisplay::unsubscribe()
{
    mesh_sub_.reset();
    image_sub_.shutdown();
}

/*
 *  private member functions
 */
void
TexturedMeshDisplay::imageCB(const msg_cp<image_t>& image)
{
    std::lock_guard<std::mutex>	lock(image_mtx_);

    cur_image_ = image;
}

void
TexturedMeshDisplay::meshCB(msg_cp<mesh_t>  mesh)
{
    std::lock_guard<std::mutex>	lock(mesh_mtx_);

    cur_mesh_  = mesh;
}

void
TexturedMeshDisplay::createTexture()
{
    std::lock_guard<std::mutex>	lock(image_mtx_);

    const auto	img = cv_bridge::toCvCopy(*cur_image_,
					  sensor_msgs::image_encodings::RGBA8);
    texture_->addMessage(img->toImageMsg());
}

void
TexturedMeshDisplay::createMesh()
{
  // Create mesh node, mesh material and manual object if not created yet.
    if (!mesh_node_)
    {
	mesh_node_.reset(scene_node_->createChildSceneNode());
	manual_object_.reset(context_->getSceneManager()
				     ->createManualObject("MeshObject"));
	mesh_node_->attachObject(manual_object_.get());

      // Create resource group named "rviz_rendering" if not existing.
	const Ogre::String resource_group_name = "rviz_rendering";
	auto&&		   rg_mgr = Ogre::ResourceGroupManager::getSingleton();
	if (!rg_mgr.resourceGroupExists(resource_group_name))
	    rg_mgr.createResourceGroup(resource_group_name);

	mesh_material_ = Ogre::MaterialManager::getSingleton().create(
			     "MeshMaterial", resource_group_name);
    }

    std::lock_guard<std::mutex>	lock(mesh_mtx_);

  // Get a transform from mesh frame to RViz display frame.
    Ogre::Vector3	position;
    Ogre::Quaternion	orientation;
    if (!context_->getFrameManager()->getTransform(cur_mesh_->header,
						   position, orientation))
	throw std::runtime_error("Error transforming from mesh frame["
				 + cur_mesh_->header.frame_id
				 + "] to current RViz frame");
    Ogre::Matrix4	transform;
    transform.makeTransform(position, Ogre::Vector3(1.0, 1.0, 1.0),
			    orientation);

  // Compute normals for all vertices.
    std::vector<Ogre::Vector3>	normals(cur_mesh_->mesh.vertices.size());
    for (const auto& triangle : cur_mesh_->mesh.triangles)
    {
	const auto	idx0 = triangle.vertex_indices[0];
	const auto	idx1 = triangle.vertex_indices[1];
	const auto	idx2 = triangle.vertex_indices[2];
	const auto	vtx0 = fromMsg(cur_mesh_->mesh.vertices[idx0]);
	const auto	vtx1 = fromMsg(cur_mesh_->mesh.vertices[idx1]);
	const auto	vtx2 = fromMsg(cur_mesh_->mesh.vertices[idx2]);
	auto		normal = (vtx1 - vtx0).crossProduct(vtx2 - vtx1);
	normal.normalise();
	normals[idx0] = normal;
	normals[idx1] = normal;
	normals[idx2] = normal;
    }

  // Add positions, normals and texture coordinates to the manual object.
    if (manual_object_->getCurrentVertexCount() ==
	cur_mesh_->mesh.vertices.size())
    {
	manual_object_->beginUpdate(0);
    }
    else
    {
	manual_object_->clear();
	manual_object_->estimateVertexCount(cur_mesh_->mesh.vertices.size());
	manual_object_->begin(mesh_material_->getName(),
			      Ogre::RenderOperation::OT_TRIANGLE_LIST,
			      "rviz_rendering");
    }

    for (size_t i = 0; i < cur_mesh_->mesh.vertices.size(); ++i)
    {
	const auto&	vertex = cur_mesh_->mesh.vertices[i];
	manual_object_->position(transform * fromMsg(vertex));
	manual_object_->normal(orientation * normals[i]);
	manual_object_->textureCoord(cur_mesh_->u[i], cur_mesh_->v[i]);
    }

    for (const auto& triangle : cur_mesh_->mesh.triangles)
    {
	manual_object_->triangle(triangle.vertex_indices[0],
				 triangle.vertex_indices[1],
				 triangle.vertex_indices[2]);
    }

    manual_object_->end();
}

void
TexturedMeshDisplay::updateMeshProperties()
{
    const auto	pass = mesh_material_->getTechnique(0)->getPass(0);
    pass->setSelfIllumination(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 0.0f));
    pass->setDiffuse(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
    pass->setAmbient(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
    pass->setSpecular(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
    pass->setShininess(64.0f);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);

    context_->queueRender();
}

void
TexturedMeshDisplay::updateCamera()
{
    const auto	pass = mesh_material_->getTechnique(0)->createPass();
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthBias(1);

    const auto	tex_state = pass->createTextureUnitState();
    tex_state->setTextureName(texture_->getName());
    tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    // tex_state->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR,
    // 				   Ogre::FO_NONE);
    tex_state->setTextureFiltering(Ogre::TFO_NONE);
    tex_state->setColourOperation(Ogre::LBO_REPLACE);  // don't accept addition
}

/*
 *  private member functions: Q_SLOTS
 */
void
TexturedMeshDisplay::updateTopic()
{
    unsubscribe();
    subscribe();
}

}  // namespace aist_visualization

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(aist_visualization::TexturedMeshDisplay,
		       rviz_common::Display)
