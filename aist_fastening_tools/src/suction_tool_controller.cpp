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
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <aist_msgs/action/suction_tool_command.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

namespace aist_fastening_tools
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
*  class SuctionToolController						*
************************************************************************/
class SuctionToolController : public rclcpp::Node
{
  private:
    using io_states_t		= ur_msgs::msg::IOStates;
    using set_io_t		= ur_msgs::srv::SetIO;
    using joint_state_t		= sensor_msgs::msg::JointState;
    using bool_t		= std_msgs::msg::Bool;
    using suction_tool_command_t= aist_msgs::action::SuctionToolCommand;
    using goal_uuid_t		= rclcpp_action::GoalUUID;
    using goal_response_t	= rclcpp_action::GoalResponse;
    using cancel_response_t	= rclcpp_action::CancelResponse;
    using callback_group_p	= rclcpp::CallbackGroup::SharedPtr;

    template <class MSG>
    using msg_p		= typename MSG::UniquePtr;
    template <class MSG>
    using pub_p		= typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using sub_p		= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using clnt_p	= typename rclcpp::Client<SRV>::SharedPtr;
    template <class ACT>
    using action_p	= typename rclcpp_action::Server<ACT>::SharedPtr;
    template <class ACT>
    using goal_cp	= std::shared_ptr<const typename ACT::Goal>;
    template <class ACT>
    using goal_handle_t	= rclcpp_action::ServerGoalHandle<ACT>;
    template <class ACT>
    using goal_handle_p	= std::shared_ptr<goal_handle_t<ACT> >;

    enum Stage	{ ACTIVE, LOOSEN, RETIGHTEN, DONE };

  public:
    SuctionToolController(const rclcpp::NodeOptions& options)	;

  private:
    goal_response_t
		goal_cb(const goal_uuid_t&,
			goal_cp<suction_tool_command_t> goal)		;
    cancel_response_t
		cancel_cb(goal_handle_p<suction_tool_command_t>)	;
    void	handle_accepted_cb(
		    goal_handle_p<suction_tool_command_t> goal_handle)	;
    void	io_states_cb(msg_p<io_states_t> states)			;
    bool	set_out_port(int port, bool state)		const	;

  private:
  // Read-only parameters
    const std::string				_driver_ns;
    const int					_in_port;
    const int					_suck_port;
    const int					_blow_port;
    const std::string				_joint_name;
    const double				_min_pos;
    const double				_max_pos;

  // IO states ubscriber and publishers
    double					_cur_pos;
    bool					_suctioned;
    const callback_group_p			_io_states_cbg;
    const sub_p<io_states_t>			_io_states_sub;
    const pub_p<joint_state_t>			_joint_state_pub;
    const pub_p<bool_t>				_suctioned_pub;

  // SetIO service client stuffs
    const callback_group_p			_set_io_cbg;
    const clnt_p<set_io_t>			_set_io_clnt;

  // Gripper command action stuffs
    const action_p<suction_tool_command_t>	_command_srv;
    goal_handle_p<suction_tool_command_t>	_current_goal_handle;
    std::mutex					_current_goal_mtx;
    rclcpp::Time				_start_time;
    rclcpp::Time				_timeout_time;
};

SuctionToolController::SuctionToolController(
    const rclcpp::NodeOptions& options)
    :rclcpp::Node("suction_gripper_controller", options),
     _driver_ns(ddynamic_reconfigure2::declare_read_only_parameter(
		    this, "driver_ns", "suction_grippers_fastening_driver")),
     _in_port(ddynamic_reconfigure2::declare_read_only_parameter(
		  this, "digital_in_port", -1)),
     _suck_port(ddynamic_reconfigure2::declare_read_only_parameter(
		    this, "digital_out_port_suck", -1)),
     _blow_port(ddynamic_reconfigure2::declare_read_only_parameter(
		    this, "digital_out_port_blow", -1)),
     _joint_name(ddynamic_reconfigure2::declare_read_only_parameter(
		    this, "joint_name", "")),
     _min_pos(ddynamic_reconfigure2::declare_read_only_parameter(
		  this, "min_position", 0.000)),
     _max_pos(ddynamic_reconfigure2::declare_read_only_parameter(
		  this, "max_position", 0.015)),
     _cur_pos(_min_pos),
     _suctioned(false),
     _io_states_cbg(create_callback_group(
			rclcpp::CallbackGroupType::MutuallyExclusive)),
     _io_states_sub(create_subscription<io_states_t>(
			_driver_ns + "/io_states", 1,
			std::bind(&SuctionToolController::io_states_cb,
				  this, std::placeholders::_1),
			create_subscription_options(_io_states_cbg))),
     _joint_state_pub(_joint_name == "" ? nullptr :
		      create_publisher<joint_state_t>("/joint_states", 1)),
     _suctioned_pub(create_publisher<bool_t>("~/suctioned", 1)),
     _set_io_cbg(create_callback_group(
		     rclcpp::CallbackGroupType::MutuallyExclusive)),
     _set_io_clnt(create_client<set_io_t>(_driver_ns + "/set_io",
					  rclcpp::ServicesQoS(), _set_io_cbg)),
     _command_srv(rclcpp_action::create_server<suction_tool_command_t>(
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
     _start_time(now()),
     _timeout_time(now())
{
    RCLCPP_INFO_STREAM(get_logger(), "controller started: driver_ns="
		       << _driver_ns << ", in_port=" << _in_port
		       << ", blow_port=" << _blow_port
		       << ", suck_port=" << _suck_port
		       << ", joint_name=" << _joint_name);
}

SuctionToolController::goal_response_t
SuctionToolController::goal_cb(const goal_uuid_t&,
			       goal_cp<suction_tool_command_t> goal)
{
    RCLCPP_INFO_STREAM(get_logger(),
		       "goal ACCEPTED: suck=" << std::boolalpha << goal->suck
		       << ", min_period=" << goal->min_period);
    return goal_response_t::ACCEPT_AND_EXECUTE;
}

SuctionToolController::cancel_response_t
SuctionToolController::cancel_cb(goal_handle_p<suction_tool_command_t>)
{
    RCLCPP_WARN_STREAM(get_logger(), "accepted request for cancelling goal");
    return cancel_response_t::ACCEPT;
}

void
SuctionToolController::handle_accepted_cb(
    goal_handle_p<suction_tool_command_t> goal_handle)
{
    const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

  // If any active goal exists, abort it.
    if (_current_goal_handle != nullptr && _current_goal_handle->is_active())
    {
	auto	result = std::make_unique<suction_tool_command_t::Result>();
	result->suctioned = _suctioned;
	_current_goal_handle->abort(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_WARN_STREAM(get_logger(), "previous goal ABORTED");
    }

  // Send suck/blow commands.
    if (!set_out_port(_suck_port,  goal_handle->get_goal()->suck) ||
	!set_out_port(_blow_port, !goal_handle->get_goal()->suck))
    {
	auto	result = std::make_unique<suction_tool_command_t::Result>();
	result->suctioned = _suctioned;
	goal_handle->abort(std::move(result));

	RCLCPP_ERROR_STREAM(get_logger(), "goal ABORTED");
	return;
    }

    _current_goal_handle = goal_handle;
    _start_time		 = now();
    _timeout_time        = (_current_goal_handle->get_goal()->timeout > 0.0 ?
                            _start_time
                            + std::chrono::duration<double>(
                                _current_goal_handle->get_goal()->timeout) :
                            rclcpp::Time::max());
}

void
SuctionToolController::io_states_cb(msg_p<io_states_t> states)
{
    const auto	current_time = now();

  // Publish joint_states.
    if (_joint_state_pub != nullptr)
    {
	auto	joint_state = std::make_unique<joint_state_t>();
	joint_state->header.stamp = current_time;
	joint_state->name.push_back(_joint_name);
	joint_state->position.push_back(_cur_pos);
	joint_state->velocity.push_back(0.0);
	joint_state->effort.push_back(0.0);
	_joint_state_pub->publish(std::move(joint_state));
    }

  // Find the state of IN port and publish its its digital IN state
  // as a flag describing the suctioned state.
    if (_in_port >= 0)
    {
	const auto state = std::find_if(states->digital_in_states.cbegin(),
					states->digital_in_states.cend(),
					[in_port=_in_port]
					(const auto& din_state)
					{ return din_state.pin == in_port; });
	if (state == states->digital_in_states.cend())
	{
	    RCLCPP_ERROR_STREAM(get_logger(),
				"no digital IN state found at port["
				<< _in_port << ']');
	    return;
	}
	_suctioned = state->state;

	auto	suctioned = std::make_unique<bool_t>();
	suctioned->data = _suctioned;
	_suctioned_pub->publish(std::move(suctioned));
    }
    else
	_suctioned = false;

  // Check if the current goal is active.
    if (!_current_goal_handle || !_current_goal_handle->is_active())
	return;

  // Update _cur_pos according to the given goal.
    _cur_pos = (_current_goal_handle->get_goal()->suck ? _max_pos : _min_pos);

    const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

  // Check if the current goal is requested to be canceled.
    if (_current_goal_handle->is_canceling())
    {
        set_out_port(_suck_port, false);        // If sucking, stop it.
	set_out_port(_blow_port, false);	// If blowing, stop it.

	auto	result = std::make_unique<suction_tool_command_t::Result>();
	result->suctioned = _suctioned;
	_current_goal_handle->canceled(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_WARN_STREAM(get_logger(), "goal CANCELED");
	return;
    }

  // Update start time if the actual state is differenct form the desired one.
    if (_suctioned != _current_goal_handle->get_goal()->suck)
	_start_time = current_time;

  // If the situation where the actual state coincides with the desired one
  // continues for the specified period, make the current goal succeeded.
    if (current_time >=
	_start_time + std::chrono::duration<double>(_current_goal_handle
                                                    ->get_goal()->min_period))
    {
	set_out_port(_blow_port, false);	// If blowing, stop it.

	auto	result = std::make_unique<suction_tool_command_t::Result>();
	result->suctioned = _suctioned;
	_current_goal_handle->succeed(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_INFO_STREAM(get_logger(), "goal SUCCEEDED["
			   << (_suctioned ? "suctioned" : "not suctioned")
			   << ']');
	return;
    }
    else if (current_time >= _timeout_time)
    {
        set_out_port(_suck_port, false);        // If sucking, stop it.
	set_out_port(_blow_port, false);	// If blowing, stop it.

	auto	result = std::make_unique<suction_tool_command_t::Result>();
	result->suctioned = _suctioned;
	_current_goal_handle->abort(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_ERROR_STREAM(get_logger(), "goal ABORTED[timeout expired]");
	return;
    }

  // Publish speed and filtered current as a feedback.
    auto feedback = std::make_unique<suction_tool_command_t::Feedback>();
    feedback->suctioned = _suctioned;
    _current_goal_handle->publish_feedback(std::move(feedback));
}

bool
SuctionToolController::set_out_port(int port, bool state) const
{
    using namespace	std::chrono_literals;

    if (port < 0)	// _blow_port may not be present
	return true;

    RCLCPP_DEBUG_STREAM(get_logger(), "set_out_port(): port="
			<< port << ", state=" << std::boolalpha << state);

    auto	req = std::make_unique<set_io_t::Request>();
    req->fun   = set_io_t::Request::FUN_SET_DIGITAL_OUT;
    req->pin   = port;
    req->state = float(state ? set_io_t::Request::STATE_ON
			     : set_io_t::Request::STATE_OFF);
    auto	future = _set_io_clnt->async_send_request(std::move(req));

    if (future.wait_for(1s) != std::future_status::ready ||
	!future.get()->success)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "no service response or error in setting DO port");
	return false;
    }

    RCLCPP_DEBUG_STREAM(get_logger(), "set_out_port(): received response");
    return true;
}
}	// namespace aist_fastening_tools

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_fastening_tools::SuctionToolController)
