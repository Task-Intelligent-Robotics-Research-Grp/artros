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
 *  \file	suction_tool_controller.cpp
 *  \brief	controller for suction tools
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <aist_msgs/action/suction_tool_command.hpp>

namespace aist_fastening_tools
{
/************************************************************************
*  class SuctionToolController						*
************************************************************************/
class SuctionToolController : public rclcpp::Node
{
  private:
    using io_states_t		= ur_msgs::msg::IOStates;
    using io_states_cp		= io_states_t::UniquePtr;
    using set_io_t		= ur_msgs::srv::SetIO;
    using joint_state_t		= sensor_msgs::msg::JointState;
    using suction_tool_command_t= aist_msgs::action::SuctionToolCommand;
    using goal_cp		= std::shared_ptr<
					const suction_tool_command_t::Goal>;
    using goal_uuid_t		= rclcpp_action::GoalUUID;
    using goal_handle_t		= rclcpp_action::
					ServerGoalHandle<suction_tool_command_t>;
    using goal_handle_p		= std::shared_ptr<goal_handle_t>;
    using goal_response_t	= rclcpp_action::GoalResponse;
    using cancel_response_t	= rclcpp_action::CancelResponse;
    using callback_group_p	= rclcpp::CallbackGroup::SharedPtr;
    template <class MSG>
    using publisher_p	 = typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using subscription_p = typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using client_p	 = typename rclcpp::Client<SRV>::SharedPtr;
    template <class ACT>
    using action_server_p= typename rclcpp_action::Server<ACT>::SharedPtr;

    enum Stage	{ ACTIVE, LOOSEN, RETIGHTEN, DONE };

  public:
    SuctionToolController(const rclcpp::NodeOptions& options)	;

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
    const std::string					_driver_ns;
    const double					_min_pos;
    const double					_max_pos;
    const double					_present_pos;

  // Joint state publisher stuffs
    const publisher_p<joint_state_t>			_joint_state_pub;

  // Gripper command action stuffs
    const action_server_p<suction_tool_command_t>	_command_srv;
    goal_handle_p					_current_goal_handle;
    std::mutex						_current_goal_mtx;
    rclcpp::Time					_last_move_time;
};

SuctionToolController::SuctionToolController(
    const rclcpp::NodeOptions& options)
    :rclcpp::Node("suction_gripper_controller", options),
     _driver_ns(ddynamic_reconfigure2::declare_read_only_parameter(
		    this, "driver_ns", "suction_grippers_fastening_driver")),
     _min_pos(ddynamic_reconfigure2::declare_read_only_parameter(
		  this, "min_position_count", 2300)),
     _max_pos(ddynamic_reconfigure2::declare_read_only_parameter(
		  this, "max_position_count", 2050)),
     _present_pos(_min_pos),
     _joint_state_pub(create_publisher<joint_state_t>("/joint_states", 1)),
     _command_srv(rclcpp_action::create_server<gripper_command_t>(
		      this, "~/gripper_cmd",
		      std::bind(&SuctionToolController::goal_cb, this,
				std::placeholders::_1, std::placeholders::_2),
		      std::bind(&SuctionToolController::cancel_cb, this,
				std::placeholders::_1),
		      std::bind(&SuctionToolController::
				handle_accepted_cb, this,
				std::placeholders::_1))),
     _current_goal_handle(nullptr),
     _current_goal_mtx(),
     _last_move_time(now())
{
    RCLCPP_INFO_STREAM(get_logger(), "controller started");
}

SuctionToolController::goal_response_t
SuctionToolController::goal_cb(const goal_uuid_t&, const goal_cp goal)
{
    RCLCPP_INFO_STREAM(get_logger(),
		       "goal ACCEPTED: position=" << goal->command.position
		       << ", max_effort=" << goal->command.max_effort);
    return goal_response_t::ACCEPT_AND_EXECUTE;
}

SuctionToolController::cancel_response_t
SuctionToolController::cancel_cb(const goal_handle_p)
{
    RCLCPP_DEBUG_STREAM(get_logger(), "accepted request for cancelling goal");
    return cancel_response_t::ACCEPT;
}

void
SuctionToolController::handle_accepted_cb(const goal_handle_p goal_handle)
{
    const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

  // If any active goal exists, abort it.
    if (_current_goal_handle != nullptr && _current_goal_handle->is_active())
    {
	auto	result = std::make_shared<gripper_command_t::Result>();
	result->position     = actual_position(_present_pos);
	result->effort	     = 0.0;
	result->stalled	     = false;
	result->reached_goal = false;
	_current_goal_handle->abort(result);
	_current_goal_handle = nullptr;

	RCLCPP_WARN_STREAM(get_logger(), "previous goal ABORTED");
    }

    _last_move_time = now();

    if (!send_move_command(goal_handle->get_goal()->command.position,
			   goal_handle->get_goal()->command.max_effort))
    {
	const auto result = std::make_shared<gripper_command_t::Result>();
	result->position     = actual_position(_present_pos);
	result->effort	     = 0.0;
	result->stalled	     = false;
	result->reached_goal = false;
	goal_handle->abort(result);

	RCLCPP_ERROR_STREAM(get_logger(), "goal ABORTED");
	return;
    }

    _current_goal_handle = goal_handle;
}

void
SuctionToolController::dynamixel_states_cb(const dynamixel_states_cp& states)
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
	const auto result = std::make_shared<gripper_command_t::Result>();
	result->stalled = false;
	_current_goal_handle->canceled(result);
	_current_goal_handle = nullptr;

	RCLCPP_WARN_STREAM(get_logger(), "goal CANCELED");
	return;
    }

    if (is_moving(state->present_velocity))
	_last_move_time = current_time;
    else if (reached_goal(state->present_position, state->present_velocity))
    {
	const auto	result = std::make_shared<gripper_command_t::Result>();
	result->position     = actual_position(state->present_position);
	result->effort	     = actual_effort(state->present_current);
	result->stalled	     = stalled(state->present_velocity);
	result->reached_goal = true;
	_current_goal_handle->succeed(result);
	_current_goal_handle = nullptr;

	RCLCPP_INFO_STREAM(get_logger(), "goal SUCCEEDED[reached goal]");
	return;
    }
    else if (stalled(state->present_velocity))
    {
	const auto	result = std::make_shared<gripper_command_t::Result>();
	result->position     = actual_position(state->present_position);
	result->effort	     = actual_effort(state->present_current);
	result->stalled	     = true;
	result->reached_goal = false;
	_current_goal_handle->succeed(result);
	_current_goal_handle = nullptr;

	RCLCPP_INFO_STREAM(get_logger(), "goal SUCCEEDED[stalled]");
	return;
    }

  // Publish speed and filtered current as a feedback.
    const auto	feedback = std::make_shared<gripper_command_t::Feedback>();
    feedback->position	   = actual_position(state->present_position);
    feedback->effort	   = actual_effort(state->present_current);
    feedback->stalled	   = stalled(state->present_velocity);
    feedback->reached_goal = reached_goal(state->present_position,
					  state->present_velocity);
    _current_goal_handle->publish_feedback(feedback);
}

bool
SuctionToolController::send_move_command(double position,
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
SuctionToolController::send_dxl_command(const std::string& addr_name,
					  int32_t value) const
{
    using namespace	std::chrono_literals;

    RCLCPP_DEBUG_STREAM(get_logger(), "send_dxl_command(): addr_name="
			<< addr_name << ", value=" << value);

    const auto	req = std::make_shared<dynamixel_command_t::Request>();
    req->id	   = _motor_id;
    req->addr_name = addr_name;
    req->value     = value;
    auto	future = _dxl_command->async_send_request(req);

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
SuctionToolController::actual_position(int pos) const
{
    return (pos - _min_pos) * _position_per_tick + _min_position;
}

double
SuctionToolController::actual_effort(int cur) const
{
    return cur * _effort_per_tick;
}

bool
SuctionToolController::is_moving(int vel) const
{
    return vel != 0;
}

bool
SuctionToolController::reached_goal(int pos, int vel) const
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
SuctionToolController::stalled(int vel) const
{
    return !is_moving(vel) && now() - _last_move_time > _stall_timeout;
}

int
SuctionToolController::goal_pos(double position) const
{
    return std::clamp(int((position - _min_position) / _position_per_tick
			  + _min_pos),
		      _max_pos, _min_pos);
}

int
SuctionToolController::goal_cur(double max_effort) const
{
    return std::clamp(int(max_effort / _effort_per_tick), -_max_cur, _max_cur);
}

}	// namespace aist_fastening_tools

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_fastening_tools::SuctionToolController)
