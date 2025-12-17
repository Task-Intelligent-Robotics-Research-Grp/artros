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

#  include <message_filters/subscriber.h>
#  include <message_filters/synchronizer.h>
#  include <message_filters/sync_policies/approximate_time.h>
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
			   // public Ogre::RenderTargetListener,
			   // public Ogre::RenderQueueListener
{
    Q_OBJECT
  private:
    template <class MSG>
    using msg_cp	= typename MSG::ConstSharedPtr;

    using topic_prop_t	= rviz_common::properties::RosTopicProperty;
    using enum_prop_t	= rviz_common::properties::EnumProperty;
    using texture_t	= rviz_default_plugins::displays::ROSImageTexture;

    using image_t	= sensor_msgs::msg::Image;
    using mesh_t	= aist_msgs::msg::TexturedMeshStamped;
    using sync_policy_t	= message_filters::sync_policies::ApproximateTime<
			      image_t, mesh_t>;
    using sync_t	= message_filters::Synchronizer<sync_policy_t>;

  public:
		TexturedMeshDisplay()					;
		~TexturedMeshDisplay()				override;

  // Overrides from Display
    void	onInitialize()					override;
    void	update(float wall_dt, float ros_dt)		override;
    void	reset()						override;
    void	setTopic(const QString& topic,
			 const QString& datatype)		override;

  protected:
  // Overrides from Display
    void	onEnable()					override;
    void	onDisable()					override;
    void	fixedFrameChanged()				override;

    void	subscribe()						;
    void	unsubscribe()						;

  private:
    void	processMessages(msg_cp<image_t> image,
				msg_cp<mesh_t> mesh)			;

    void	createTexture()						;
    void	createMesh()						;

    void	updateMeshProperties()					;
    void	updateCamera()						;

  protected Q_SLOTS:
    void	updateTopic()						;

  private:
    std::unique_ptr<enum_prop_t>	image_transport_property_;
    std::unique_ptr<topic_prop_t>	image_topic_property_;
    std::unique_ptr<topic_prop_t>	mesh_topic_property_;

    std::unique_ptr<image_transport::ImageTransport>		it_;
    std::shared_ptr<image_transport::SubscriberFilter>		image_sub_;
    std::shared_ptr<message_filters::Subscriber<mesh_t> >	mesh_sub_;
    std::shared_ptr<sync_t>					sync_;

    msg_cp<image_t>			cur_image_;
    msg_cp<mesh_t>			cur_mesh_;
    std::mutex				msg_mtx_;

    std::unique_ptr<texture_t>		texture_;
    std::unique_ptr<Ogre::SceneNode>	mesh_node_;
    std::unique_ptr<Ogre::ManualObject>	manual_object_;
    Ogre::MaterialPtr			mesh_material_;
};

}  // namespace aist_visualization
