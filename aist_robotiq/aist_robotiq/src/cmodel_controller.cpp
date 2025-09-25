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
// Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
//
/*!
 *  \file	cmodel_controller.cpp
 *  \brief	controller for Robotiq grippers
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <aist_robotiq_msgs/msg/cmodel_status.hpp>
#include <aist_robotiq_msgs/msg/cmodel_command.hpp>
#include <aist_robotiq_msgs/srv/set_velocity.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure.hpp>

namespace aist_robotiq
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
*  class CModelController						*
************************************************************************/
class CModelController : public rclcpp::Node
{
  private:
    using cmodel_status_t   = aist_robotiq_msgs::msg::CModelStatus;
    using cmodel_status_cp  = cmodel_status_t::UniquePtr;
    using cmodel_command_t  = aist_robotiq_msgs::srv::CModelCommand;
    using joint_state_t	    = sensor_msgs::msg::JointState;
    using gripper_command_t = control_msgs::action::GripperCommand;
    using goal_cp	    = std::shared_ptr<const gripper_command_t::Goal>;
    using goal_uuid_t	    = rclcpp_action::GoalUUID;
    using goal_handle_t	    = rclcpp_action::
				  ServerGoalHandle<gripper_command_t>;
    using goal_handle_p	    = std::shared_ptr<goal_handle_t>;
    using goal_response_t   = rclcpp_action::GoalResponse;
    using cancel_response_t = rclcpp_action::CancelResponse;

    using callback_group_p  = rclcpp::CallbackGroup::SharedPtr;
    template <class MSG>
    using publisher_p	    = typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using subscription_p    = typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using service_p	    = typename rclcpp::Service<SRV>::SharedPtr;
    template <class ACT>
    using action_server_p   = typename rclcpp_action::Server<ACT>::SharedPtr;

  public:
    CModelController(const rclcpp::NodeOptions& options)	;

  private:
    goal_response_t
		goal_cb(const goal_uuid_t&, const goal_cp goal)		;
    cancel_response_t
		cancel_cb(const goal_handle_p)				;
    void	handle_accepted_cb(const goal_handle_p goal_handle)	;
    void	dynamixel_states_cb(const dynamixel_states_cp& states)	;

    bool	send_move_command(double position,
				  double max_effort)		const	;
    bool	send_dxl_command(const std::string& addr_name,
				 int32_t value)			const	;

    double	actual_position(int pos)			const	;
    double	actual_effort(int cur)				const	;
    bool	is_moving(int vel)				const	;
    bool	reached_goal(int pos, int vel)			const	;
    bool	stalled(int vel)				const	;
    int		goal_pos(double position)			const	;
    int		goal_cur(double max_effort)			const	;

  private:
  // Read-only parameters
    const double				_min_position;
    const double				_max_position;
    const double				_min_velocity;
    const double				_max_velocity;
    const double				_min_effort;
    const double				_max_effort;
    const std::string				_joint_name;

  // Publisher for JointState
    const publisher_p<joint_state_t>		_joint_state_pub;

  // Service for setting velocity
    double					_velocity;
    const service_p<set_velocity_t>		_set_velocity_srv;

  // Publisher for command to the driver
    const publisher_p<cmodel_command_t>		_cmodel_command_pub;
    int						_goal_r_pr;

  // Subscriber for Status from the driver
    const callback_group_p			_cmodel_status_cbg;
    const subscription_p<cmodel_status_t>	_cmodel_status_sub;

    const double				_position_per_tick;
    const double				_effort_per_tick;
    const rclcpp::Duration			_stall_timeout;

  // Gripper command action stuffs
    const action_server_p<gripper_command_t>	_command_srv;
    goal_handle_p				_current_goal_handle;
    std::mutex					_current_goal_mtx;
    rclcpp::Time				_last_move_time;
};

CModelController::CModelController(const rclcpp::NodeOptions& options)
    :rclcpp::Node("cmodel_controller", options),
     _min_position(ddynamic_reconfigure2::declare_read_only_parameter(
		       this, "min_position", 0.810)),
     _max_position(ddynamic_reconfigure2::declare_read_only_parameter(
		       this, "max_position", 0.000)),
     _min_velocity(ddynamic_reconfigure2::declare_read_only_parameter(
		       this, "min_velocity", 0.013)),
     _max_velocity(ddynamic_reconfigure2::declare_read_only_parameter(
		       this, "max_velocity", 0.100)),
     _min_effort(ddynamic_reconfigure2::declare_read_only_parameter(
		     this, "mix_effort", 40.0)),
     _max_effort(ddynamic_reconfigure2::declare_read_only_parameter(
		     this, "max_effort", 100.0)),
     _joint_name(ddynamic_reconfigure2::declare_read_only_parameter(
		     this, "joint_name", "finger_joint")),
     _joint_state_pub(create_publisher<joint_state_t>("/joint_states", 1)),
     _velocity(0.5*(_min_velocity + _max_velocity)),
     _set_velocity_srv(create_service<set_velocity_t>(
			   "~/set_velocity",
			   std::bind(&CModelController::set_velocity_cb,
				     this,
				     std::placeholders::_1,
				     std::placeholders::_2))),
     _cmodel_command_pub(create_publisher<cmodel_command_t>("~/command", 1)),
     _goal_r_pr(0),
     _cmodel_status_cbg(create_callback_group(
			    rclcpp::CallbackGroupType::MutuallyExclusive)),
     _cmodel_status_sub(create_subscription<cmodel_status_t>(
			    "~/status", 1,
			    std::bind(&CModelController::cmodel_status_cb,
				      this, std::placeholders::_1),
			    create_subscription_options(_cmodel_status_cbg))),
     _position_per_tick((_max_position - _min_position)/(_max_pos - _min_pos)),
     _effort_per_tick(_max_effort/_max_cur),
     _stall_timeout(std::chrono::duration<double>(
			ddynamic_reconfigure2::declare_read_only_parameter(
			    this, "stall_timeout", 1.0))),
     _dxl_states_sub(create_subscription<dynamixel_states_t>(
			 _driver_ns + "/dynamixel_state", 1,
			 std::bind(
			     &CModelController::dynamixel_states_cb,
			     this, std::placeholders::_1))),
     _dxl_command_cbg(),
     _dxl_command(create_client<dynamixel_command_t>(
		      _driver_ns + "/dynamixel_command",
		      rclcpp::ServicesQoS(), _dxl_command_cbg)),
     _present_pos(_min_pos),
     _command_srv(rclcpp_action::create_server<gripper_command_t>(
		      this, "~/gripper_cmd",
		      std::bind(&CModelController::goal_cb, this,
				std::placeholders::_1, std::placeholders::_2),
		      std::bind(&CModelController::cancel_cb, this,
				std::placeholders::_1),
		      std::bind(&CModelController::
				handle_accepted_cb, this,
				std::placeholders::_1))),
     _current_goal_handle(nullptr),
     _current_goal_mtx(),
     _last_move_time(now())
{
    using namespace	std::chrono_literals;

    if (!_dxl_command->wait_for_service(1s))
	throw std::runtime_error("service not available");

    RCLCPP_INFO_STREAM(get_logger(), "controller started");
}

CModelController::goal_response_t
CModelController::goal_cb(const goal_uuid_t&, const goal_cp goal)
{
    RCLCPP_INFO_STREAM(get_logger(),
		       "goal ACCEPTED: position=" << goal->command.position
		       << ", max_effort=" << goal->command.max_effort);
    return goal_response_t::ACCEPT_AND_EXECUTE;
}

CModelController::cancel_response_t
CModelController::cancel_cb(const goal_handle_p)
{
    RCLCPP_DEBUG_STREAM(get_logger(), "accepted request for cancelling goal");
    return cancel_response_t::ACCEPT;
}

void
CModelController::handle_accepted_cb(const goal_handle_p goal_handle)
{
    const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

  // If any active goal exists, abort it.
    if (_current_goal_handle != nullptr && _current_goal_handle->is_active())
    {
	auto	result = std::make_unique<gripper_command_t::Result>();
	result->position     = actual_position(_present_pos);
	result->effort	     = 0.0;
	result->stalled	     = false;
	result->reached_goal = false;
	_current_goal_handle->abort(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_WARN_STREAM(get_logger(), "previous goal ABORTED");
    }

    _last_move_time = now();

    if (!send_move_command(goal_handle->get_goal()->command.position,
			   goal_handle->get_goal()->command.max_effort))
    {
	auto	result = std::make_unique<gripper_command_t::Result>();
	result->position     = actual_position(_present_pos);
	result->effort	     = 0.0;
	result->stalled	     = false;
	result->reached_goal = false;
	goal_handle->abort(std::move(result));

	RCLCPP_ERROR_STREAM(get_logger(), "goal ABORTED");
	return;
    }

    _current_goal_handle = goal_handle;
}

void
CModelController::dynamixel_states_cb(const dynamixel_states_cp& states)
{
  // Find a dynamixel state with my motor ID.
    const auto	state = std::find_if(states->dynamixel_state.begin(),
				     states->dynamixel_state.end(),
				     [motor_id=_motor_id](const auto& state)
				     { return state.id == motor_id; });
    if (state == states->dynamixel_state.end())
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "no motors with ID[" << int(_motor_id)
			    << "] found in incoming dynamixel state list!");
	return;
    }

  // Keep present position of Dynamixel.
    _present_pos = state->present_position;

  // Publish joinst state.
    const auto	current_time = now();
    auto	joint_state = std::make_unique<joint_state_t>();
    joint_state->header.stamp = current_time;
    joint_state->name.push_back(state->name + "_finger_joint");
    joint_state->position.push_back(actual_position(state->present_position));
    joint_state->velocity.push_back(0.0);
    joint_state->effort.push_back(actual_effort(state->present_current));
    _joint_state_pub->publish(std::move(joint_state));

  // Check if the current goal is active.
    if (!_current_goal_handle || !_current_goal_handle->is_active())
	return;

    const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

  // Check if the current goal is requested to be cancelled.
    if (_current_goal_handle->is_canceling())
    {
	auto	result = std::make_unique<gripper_command_t::Result>();
	result->stalled = false;
	_current_goal_handle->canceled(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_WARN_STREAM(get_logger(), "goal CANCELED");
	return;
    }

    if (is_moving(state->present_velocity))
	_last_move_time = current_time;
    else if (reached_goal(state->present_position, state->present_velocity))
    {
	auto	result = std::make_unique<gripper_command_t::Result>();
	result->position     = actual_position(state->present_position);
	result->effort	     = actual_effort(state->present_current);
	result->stalled	     = stalled(state->present_velocity);
	result->reached_goal = true;
	_current_goal_handle->succeed(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_INFO_STREAM(get_logger(), "goal SUCCEEDED[reached goal]");
	return;
    }
    else if (stalled(state->present_velocity))
    {
	auto	result = std::make_unique<gripper_command_t::Result>();
	result->position     = actual_position(state->present_position);
	result->effort	     = actual_effort(state->present_current);
	result->stalled	     = true;
	result->reached_goal = false;
	_current_goal_handle->succeed(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_INFO_STREAM(get_logger(), "goal SUCCEEDED[stalled]");
	return;
    }

  // Publish speed and filtered current as a feedback.
    auto	feedback = std::make_unique<gripper_command_t::Feedback>();
    feedback->position	   = actual_position(state->present_position);
    feedback->effort	   = actual_effort(state->present_current);
    feedback->stalled	   = stalled(state->present_velocity);
    feedback->reached_goal = reached_goal(state->present_position,
					  state->present_velocity);
    _current_goal_handle->publish_feedback(std::move(feedback));
}

bool
CModelController::send_move_command(double position,
					   double max_effort) const
{
    const auto	pos = goal_pos(position);
    auto	cur = goal_cur(max_effort);
    if (std::abs(cur) < _min_cur)
	cur = (pos > _present_pos ? _min_cur : -_min_cur);

    return send_dxl_command("Goal_Current",  cur) &&
	   send_dxl_command("Goal_Position", pos);
}

bool
CModelController::send_dxl_command(const std::string& addr_name,
					  int32_t value) const
{
    using namespace	std::chrono_literals;

    RCLCPP_DEBUG_STREAM(get_logger(), "send_dxl_command(): addr_name="
			<< addr_name << ", value=" << value);

    auto	req = std::make_unique<dynamixel_command_t::Request>();
    req->id	   = _motor_id;
    req->addr_name = addr_name;
    req->value     = value;
    auto	future = _dxl_command->async_send_request(std::move(req));

    if (future.wait_for(1s) != std::future_status::ready ||
	!future.get()->comm_result)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "no service response or communication error");
	return false;
    }

    RCLCPP_DEBUG_STREAM(get_logger(), "send_dxl_command(): received response");
    return true;
}

double
CModelController::actual_position(int pos) const
{
    return (pos - _min_pos) * _position_per_tick + _min_position;
}

double
CModelController::actual_effort(int cur) const
{
    return cur * _effort_per_tick;
}

bool
CModelController::is_moving(int vel) const
{
    return vel != 0;
}

bool
CModelController::reached_goal(int pos, int vel) const
{
    RCLCPP_DEBUG_STREAM(get_logger(), "*** pos=" << pos << ", goal_pos="
			<< goal_pos(_current_goal_handle
				    ->get_goal()->command.position)
			<< ", vel=" << vel);
    return !is_moving(vel) &&
	   std::abs(pos - goal_pos(_current_goal_handle
				   ->get_goal()->command.position)) <= 1;
}

bool
CModelController::stalled(int vel) const
{
    return !is_moving(vel) && now() - _last_move_time > _stall_timeout;
}

int
CModelController::goal_pos(double position) const
{
    return std::clamp(int((position - _min_position) / _position_per_tick
			  + _min_pos),
		      _max_pos, _min_pos);
}

int
CModelController::goal_cur(double max_effort) const
{
    return std::clamp(int(max_effort / _effort_per_tick), -_max_cur, _max_cur);
}

}	// namespace aist_robotiq

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_robotiq::CModelController)
