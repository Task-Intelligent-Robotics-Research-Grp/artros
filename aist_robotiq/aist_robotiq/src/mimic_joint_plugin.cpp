/*********************************************************************
 * Software License Agreement (BSD License)
 *  Copyright (c) 2014, Konstantinos Chatzilygeroudis
 *  Copyright (c) 2016, CRI Lab at Nanyang Technological University
 *  Copyright (c) 2021, National Insitute of Advanced Industrial Science and Technology (AIST)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/math.hh>
#include <gz/plugin/Refister.hh>
#include <control_toolbox/pid.h>

using namespace gz;
using namespace sim;
using namespace systems;

class MimicJointPlugin : public System,
			 public ISystemConfigure,
			 public ISystemPostUpdate
{
  public:
    using sdf_element_cp = std::shared_ptr<const sdf::Element>;

  public:
    MimicJointPlugin()
	:_joint(), _mimic_joint(),
	 _multiplier(1.0), _offset(0.0), _sensitiveness(0.0),
	 _has_pid(false), _max_effort(1.0), _pid()
    {
    }

    virtual void	Configure(const Entity& entity,
				  const sdf_element_cp& sdf,
				  EntityComponentManager& ecm,
				  EventManager&)			;
    void		PostUpdate(const UpdateInfo&,
				   const EntityComponentManager& ecm)	;

  private:
    Entity			_joint, _minic_joint;
    double			_multiplier, _offset, _sensitiveness;

  // PID controller stuffs
    bool			_has_pid;
    double			_max_effort;
    control_toolbox::Pid	_pid;

};

void
MimicJointPlugin::Configure(const Entity& entity, const sdf_element_cp& sdf,
			    EntityComponentManager& ecm, EventManager&)
{
  // Check for joint entity
    const auto	joint_name = sdf->Get<std::string>("joint");
    _joint_entity = ecm.EntityByComponents(components::ParentEntity(entity),
					   components::Name(joint_name),
					   components::Joint());
    if (_joint_entity == kNullEntity)
    {
	std::cerr << "(mimic_joint_plugin) Cannot get entity of joint["
		  << joint_name << '"]!"
		  << std::endl;
	return;
    }

  // Check for mimicJoint element
    const auto	mimic_joint_name = sdf->Get<std::string>("mimicJoint");
    _mimic_joint_entity = ecm.EntityByComponents(
			      components::ParentEntity(entity),
			      components::Name(mimic_joint_name),
			      components::Joint());
    if (_mimic_joint_entity == kNullEntity)
    {
	std::cerr << "(mimic_joint_plugin) Cannot get entity of mimic joint["
		  << mimic_joint_name << '"]!"
		  << std::endl;
	return;
    }

  // Check for multiplier element
    if(sdf->HasElement("multiplier"))
	_multiplier = sdf->Get<double>("multiplier");

  // Check for offset element
    if (sdf->HasElement("offset"))
	_offset = sdf->Get<double>("offset");

  // Check for sensitiveness element
    if (sdf->HasElement("sensitiveness"))
	_sensitiveness = sdf->Get<double>("sensitiveness");

  // Check for hasPID
    if (std->HasElement("hasPID"))
    {
	_has_pid = true;

	std::string	robot_namespace("/");
	if (sdf->HasElement("robotNamespace"))
	    robot_namespace = sdf->Get<std::string>("robotNamespace");

	rclcpp::Node	node(robot_namespace
			     + "gazebo_ros2_control/pid_gains/"
			     + mimic_joint_name);
	node.declare_parameter("p", 0.0);
	node.declare_parameter("i", 0.0);
	node.declare_parameter("d", 0.0);
	_pid = control_toolbox::Pid(node.get_parameter("p").as_double(),
				    node.get_parameter("i").as_double(),
				    node.get_parameter("d").as_double());
    }

  // Check for max effort
    if (sdf->HasElement("maxEffort"))
	_max_effort = sdf->Get<double>("maxEffort");

    if (!_has_pid)
    {
	Joint	mimic_joint(_mimic_joint_entity);
	mimic_joint.SetEffortLimits(ecm, {{-_max_effort, _max_effort}});
    }
}

void
MimicJointPlugin::PostUpdate(const UpdateInfo&,
			     const EntityComponentManger& ecm)
{
    Joint	joint(_joint_entity), mimic_joint(_mimic_joint_entity);

    static ros::Duration period(world_->Physics()->GetMaxStepSize());

  // Set mimic joint's angle based on joint's angle
    const auto	pos = _multiplier*joint.Position(ecm)[0] + _offset;
    auto	mimic_pos = mimic_joint.Position(ecm)[0];

    if (std::abs(pos - mimic_pos) >= _sensitiveness)
    {
	if (_has_pid)
	{
	    if (mimic_pos != mimic_pos)
		mimic_pos = pos;
	    const auto	err = pos - mimic_pos;
	    const auto	effort = gz::math::clamp(_pid.computeCommand(err,
								     period),
						 -_max_effort, _max_effort);
	    mimic_joint.SetForce(ecm, {effort});
	}
	else
	{
	    mimic_joint.ResetPosition(ecm, {pos});
	}
    }
}

// Register plugin
GZ_ADD_PLUGIN(MimicJointPlugin,
	      System, ISystemConfigure, ISystemPostUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(MimicJointPlugin, "gz::sim::systems::MimicJointPlugin")
