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
  \file		textured_mesh_display.hpp
  \author	Toshio Ueshiba
*/
#pragma once

#ifndef Q_MOC_RUN
#  include <QObject>
#  include <Ogre.h>

#  include <mutex>

#  include <image_transport/image_transport.hpp>
#  include <image_transport/subscriber_filter.hpp>

#  include <rviz_common/display.hpp>
#  include <rviz_common/properties/enum_property.hpp>
#  include <rviz_common/properties/ros_topic_property.hpp>
#  include <rviz_default_plugins/displays/image/ros_image_texture.hpp>
#  include <aist_msgs/msg/textured_mesh_stamped.hpp>

#endif  // Q_MOC_RUN

#include <QMap>
#include <QString>

namespace aist_visualization
{
/************************************************************************
*  class TexturedMeshDisplay						*
************************************************************************/
class TexturedMeshDisplay: public rviz_common::Display
{
    Q_OBJECT
  private:
    template <class MSG>
    using msg_cp	= typename MSG::ConstSharedPtr;
    using image_t	= sensor_msgs::msg::Image;
    using mesh_t	= aist_msgs::msg::TexturedMeshStamped;

    using topic_prop_t	= rviz_common::properties::RosTopicProperty;
    using texture_t	= rviz_default_plugins::displays::ROSImageTexture;

  public:
		TexturedMeshDisplay()					;
		~TexturedMeshDisplay()				override;

  // Overrides from Display
    void	onInitialize()					override;
    void	update(float wall_dt, float ros_dt)		override;
    void	reset()						override;

  protected:
  // Overrides from Display
    void	onEnable()					override;
    void	onDisable()					override;
    void	fixedFrameChanged()				override;

    void	subscribe()						;
    void	unsubscribe()						;

  private:
    void	imageCB(const msg_cp<image_t>& image)			;
    void	meshCB(msg_cp<mesh_t> mesh)				;

    void	createTexture()						;
    void	createMesh()						;

    void	updateMeshProperties()					;
    void	updateCamera()						;

  protected Q_SLOTS:
    void	updateTopic()						;

  private:
    std::unique_ptr<topic_prop_t>			image_topic_property_;
    std::unique_ptr<topic_prop_t>			mesh_topic_property_;
    std::unique_ptr<image_transport::ImageTransport>	it_;
    image_transport::Subscriber				image_sub_;
    rclcpp::Subscription<mesh_t>::SharedPtr		mesh_sub_;

    msg_cp<image_t>					cur_image_;
    msg_cp<mesh_t>					cur_mesh_;
    std::mutex						image_mtx_;
    std::mutex						mesh_mtx_;

    std::unique_ptr<texture_t>				texture_;
    std::unique_ptr<Ogre::SceneNode>			mesh_node_;
    std::unique_ptr<Ogre::ManualObject>			manual_object_;
    Ogre::MaterialPtr					mesh_material_;
};

}  // namespace aist_visualization
