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
 *  \file	screw_tool_controller.cpp
 *  \brief	controller for screw tools
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <dynamixel_workbench_msgs/msg/dynamixel_state_list.hpp>
#include <dynamixel_workbench_msgs/srv/dynamixel_command.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include <aist_msgs/action/screw_tool_command.hpp>
#include <aist_msgs/msg/screw_tool_status.hpp>
#include <aist_utility/butterworth_lpf.hpp>

namespace aist_fastening_tools
{
/************************************************************************
*  static functions							*
************************************************************************/
static int32_t
target_speed(double speed)
{
    speed = std::clamp(speed, -1.0, 1.0);

    return (speed >= 0.0 ? int32_t(1023*speed) : 1024 - int32_t(1023*speed));
}

static double
get_normalized(int32_t value)
{
    value = std::clamp(value, 0, 2047);
    return (value < 1024 ? value : 1024 - value) / 1023.0;
}

/************************************************************************
*  class ScrewToolController						*
************************************************************************/
class ScrewToolController : public rclcpp::Node
{
  private:
    using dynamixel_states_t	= dynamixel_workbench_msgs::msg::
					DynamixelStateList;
    using dynamixel_states_cp	= dynamixel_states_t::UniquePtr;
    using dynamixel_command_t	= dynamixel_workbench_msgs::srv::
					DynamixelCommand;
    using screw_tool_status_t	= aist_msgs::msg::ScrewToolStatus;
    using screw_tool_command_t	= aist_msgs::action::ScrewToolCommand;
    using goal_cp		= std::shared_ptr<
					const screw_tool_command_t::Goal>;
    using goal_uuid_t		= rclcpp_action::GoalUUID;
    using goal_handle_t		= rclcpp_action::
					ServerGoalHandle<screw_tool_command_t>;
    using goal_handle_p		= std::shared_ptr<goal_handle_t>;
    using goal_response_t	= rclcpp_action::GoalResponse;
    using cancel_response_t	= rclcpp_action::CancelResponse;
    using ddynamic_reconfigure_t= ddynamic_reconfigure2::DDynamicReconfigure;
    using filter_t		= aist_utility::ButterworthLPF<double, double>;

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
    ScrewToolController(const rclcpp::NodeOptions& options)		;

  private:
    goal_response_t
		goal_cb(const goal_uuid_t& uuid, const goal_cp goal)	;
    cancel_response_t
		cancel_cb(const goal_handle_p goal_handle)		;
    void	handle_accepted_cb(const goal_handle_p goal_handle)	;
    void	dynamixel_states_cb(const dynamixel_states_cp& states)	;

    bool	is_satisfied(double ratio, double max_ratio,
			     const rclcpp::Duration& min_period)	;
    bool	send_dynamixel_command(const std::string& addr_name,
				       int32_t value)			;
    void	set_period(rclcpp::Duration& period, double sec)	;
    void	set_filter_half_order(int half_order)			;
    void	set_filter_cutoff_frequency(double cutoff_frequency)	;

  private:
  // Basic stuffs
    const std::string				_driver_ns;
    const uint8_t				_motor_id;
    Stage					_stage;
    rclcpp::Time				_start_time;

  // Dynamixel driver stuffs
    const subscription_p<dynamixel_states_t>	_dynamixel_states_sub;
    const client_p<dynamixel_command_t>		_dynamixel_command;

  // Status publishment stuffs
    const publisher_p<screw_tool_status_t>	_status_pub;

  // Action stuffs
    const action_server_p<screw_tool_command_t>	_command_srv;
    goal_handle_p				_current_goal_handle;

  // Parameters
    ddynamic_reconfigure2::DDynamicReconfigure	_ddr;
    rclcpp::Duration				_loosen_period;      // period before retighten
    double					_max_stall_speed;
    rclcpp::Duration				_min_stall_period;
    double					_max_noload_current;
    rclcpp::Duration				_min_noload_period;

  // Current filtering stuffs
    const rclcpp::Duration			_control_period;
    double					_current;
    filter_t					_filter;
};

ScrewToolController::ScrewToolController(const rclcpp::NodeOptions& options)
    :rclcpp::Node("screw_tool_controller", options),
     _driver_ns(ddynamic_reconfigure2::
		declare_read_only_parameter<std::string>(this, "driver_ns",
							 "screw_tool_driver")),
     _motor_id(ddynamic_reconfigure2::
	       declare_read_only_parameter<int>(this, "motor_id", 1)),
     _stage(DONE),
     _start_time(),
     _dynamixel_states_sub(
	 create_subscription<dynamixel_states_t>(
	     _driver_ns + "/dynamixel_state", 1,
	     std::bind(&ScrewToolController::dynamixel_states_cb,
		       this, std::placeholders::_1))),
     _dynamixel_command(create_client<dynamixel_command_t>(
			    _driver_ns + "/dynamixel_command")),
     _status_pub(create_publisher<screw_tool_status_t>("~/status", 1)),
     _command_srv(rclcpp_action::create_server<screw_tool_command_t>(
		      this, "~/command",
		      std::bind(&ScrewToolController::goal_cb, this,
				std::placeholders::_1, std::placeholders::_2),
		      std::bind(&ScrewToolController::cancel_cb, this,
				std::placeholders::_1),
		      std::bind(&ScrewToolController::handle_accepted_cb, this,
				std::placeholders::_1))),
     _current_goal_handle(nullptr),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _loosen_period(std::chrono::milliseconds(1000)),
     _max_stall_speed(0.01),
     _min_stall_period(std::chrono::milliseconds(500)),
     _max_noload_current(0.3),
     _min_noload_period(std::chrono::milliseconds(500)),
     _control_period(
	 std::chrono::milliseconds(
	     ddynamic_reconfigure2::
	     declare_read_only_parameter<int>(this, "control_period", 10))),
     _current(0.0),
     _filter(2, 7.0*_control_period.seconds())
{
    using namespace	std::placeholders;

  // Setup ddynamic_reconfigure server.
    _ddr.registerVariable<double>(
	"control parameters.loosen_period",
	_loosen_period.seconds(),
	std::bind(&ScrewToolController::set_period, this, _loosen_period, _1),
	"Period of loosening before retightening",
	{0.1, 5.0});
    _ddr.registerVariable<double>(
	"control parameters.max_stall_speed",
	&_max_stall_speed,
	"Maximum ratio of speed to be judged as stalled",
	{0.0, 0.05});
    _ddr.registerVariable<double>(
	"control parameters.min_stall_period",
	_min_stall_period.seconds(),
	std::bind(&ScrewToolController::set_period,
		  this, _min_stall_period, _1),
	"Minimum period required to be judged as stalled",
	{0.1, 1.0});
    _ddr.registerVariable<double>(
	"control parameters.max_noload_current",
	&_max_noload_current,
	"Maximum ratio of current to be judged as unloaded",
	{0.0, 1.0});
    _ddr.registerVariable<double>(
	"control parameters.min_noload_period",
	_min_noload_period.seconds(),
	std::bind(&ScrewToolController::set_period,
		  this, _min_noload_period, _1),
	"Minimum period required to be judged as unloaded",
	{0.1, 1.0});
    _ddr.registerVariable<int>(
	"filtering parameters.filter_half_order",
	_filter.half_order(),
	std::bind(&ScrewToolController::set_filter_half_order, this, _1),
	"Half order of current low pass filter",
	{1, 5});
    _ddr.registerVariable<double>(
	"filtering parameters.filter_cutoff_frequency",
	_filter.cutoff()/_control_period.seconds(),
	std::bind(&ScrewToolController::set_filter_cutoff_frequency, this, _1),
	"Cutoff frequency of current low pass filter",
	{1, 30});

    RCLCPP_INFO_STREAM(get_logger(),
		       "controller started with motor ID["
		       << int(_motor_id) << ']');
}

ScrewToolController::goal_response_t
ScrewToolController::goal_cb(const goal_uuid_t& uuid, const goal_cp goal)
{
    RCLCPP_INFO_STREAM(get_logger(),
		       "goal ACCEPTED: "
		       << (goal->speed > 0 ? "tighten" : "loosen")
		       << " with speed=" << goal->speed);
    return goal_response_t::ACCEPT_AND_EXECUTE;
}

ScrewToolController::cancel_response_t
ScrewToolController::cancel_cb(const goal_handle_p goal_handle)
{
    RCLCPP_INFO_STREAM(get_logger(), "goal PREEMPTED");

    send_dynamixel_command("Moving_Speed",  0);
    send_dynamixel_command("Torque_Enable", 0);

    return cancel_response_t::ACCEPT;
}

void
ScrewToolController::handle_accepted_cb(const goal_handle_p goal_handle)
{
    if (_current_goal_handle != nullptr && _current_goal_handle->is_active())
    {
	RCLCPP_WARN_STREAM(get_logger(), "previous goal ABORTED");

	auto	result = std::make_shared<screw_tool_command_t::Result>();
	result->stalled = false;
	_current_goal_handle->abort(result);
    }

    _current_goal_handle = goal_handle;
    send_dynamixel_command("Torque_Enable", 1);
    send_dynamixel_command("Moving_Speed",
			   target_speed(
			       _current_goal_handle->get_goal()->speed));
}

void
ScrewToolController::dynamixel_states_cb(const dynamixel_states_cp& states)
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

  // Read current value.
    _current = get_normalized(state->present_current);

  // Apply low-pass filter to the current and publish tool status.
    screw_tool_status_t	status;
    status.header.stamp	= get_clock()->now();
    status.speed	= get_normalized(state->present_velocity);
    status.current	= _filter.filter(_current);
    _status_pub->publish(status);

  // Check if an active goal is available.
    if (!_current_goal_handle || !_current_goal_handle->is_active())
	return;

  // Publish speed and filtered current as a feedback.
    auto	feedback = std::make_shared<screw_tool_command_t::Feedback>();
    feedback->speed   = status.speed;
    feedback->current = status.current;
    _current_goal_handle->publish_feedback(feedback);

    if (const auto goal = _current_goal_handle->get_goal(); goal->speed > 0.0)
	switch (_stage)
	{
	  case ACTIVE:
	    if (is_satisfied(feedback->speed,
			     _max_stall_speed, _min_stall_period))
	    {
		if (goal->retighten)
		{
		    send_dynamixel_command("Moving_Speed", target_speed(0.0));
		    rclcpp::sleep_for(std::chrono::milliseconds(100));

		    RCLCPP_INFO_STREAM(get_logger(), "slightly loosen screw");

		    send_dynamixel_command("Moving_Speed",
					   target_speed(-goal->speed));
		    _stage	= LOOSEN;
		    _start_time = get_clock()->now();
		}
		else
		    _stage = DONE;
	    }
	    break;
	  case LOOSEN:
	    if (get_clock()->now() - _start_time > _loosen_period)
	    {
		send_dynamixel_command("Moving_Speed", target_speed(0.0));
		rclcpp::sleep_for(std::chrono::milliseconds(100));

		RCLCPP_INFO_STREAM(get_logger(), "retighten screw");

		send_dynamixel_command("Moving_Speed",
				       target_speed(goal->speed));
		_stage	= RETIGHTEN;
		_start_time = get_clock()->now();
	    }
	    break;
	  case RETIGHTEN:
	    if (is_satisfied(feedback->speed,
			     _max_stall_speed, _min_stall_period))
		_stage = DONE;
	    break;
	  default:
	    break;
	}
    else if (is_satisfied(status.current,
			  _max_noload_current, _min_noload_period))
	_stage = DONE;

    if (_stage == DONE)
    {
	send_dynamixel_command("Moving_Speed", target_speed(0.0));
	send_dynamixel_command("Enable_Torque", 0);

	auto	result = std::make_shared<screw_tool_command_t::Result>();
	result->stalled = true;
	_current_goal_handle->succeed(result);

	RCLCPP_INFO_STREAM(get_logger(), "goal SUCCEEDED");
    }
}

bool
ScrewToolController::is_satisfied(double ratio, double max_ratio,
				  const rclcpp::Duration& min_period)
{
    if (std::abs(ratio) > max_ratio)
	_start_time = get_clock()->now();

    return (get_clock()->now() - _start_time > min_period);
}

bool
ScrewToolController::send_dynamixel_command(const std::string& addr_name,
					    int32_t value)
{
    auto	req = std::make_shared<dynamixel_command_t::Request>();
    req->id	   = _motor_id;
    req->addr_name = addr_name;
    req->value     = value;
    auto	future = _dynamixel_command->async_send_request(req);
    future.wait();

    return future.get()->comm_result;
}

void
ScrewToolController::set_period(rclcpp::Duration& period, double sec)
{
    period = rclcpp::Duration(std::chrono::duration<double>(sec));
}

void
ScrewToolController::set_filter_half_order(int half_order)
{
    _filter.initialize(half_order, _filter.cutoff());
    _filter.reset(_current);
}

void
ScrewToolController::set_filter_cutoff_frequency(double cutoff_frequency)
{
    _filter.initialize(_filter.half_order(),
		       cutoff_frequency * _control_period.seconds());
    _filter.reset(_current);
}
}	// namespace aist_fastening_tools
