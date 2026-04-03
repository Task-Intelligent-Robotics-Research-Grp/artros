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
 *  \file       robotiq_3f_controller.cpp
 *  \brief	controller for Robotiq-3F grippers
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <aist_robotiq_msgs/msg/c_model_status.hpp>
#include <aist_robotiq_msgs/msg/c_model_command.hpp>
#include <aist_robotiq_msgs/srv/set_velocity.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

namespace aist_robotiq
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> inline std::array<T, 4>
array4_from_vector(const std::vector<T>& v)
{
    if (v.size() != 4)
        throw std::runtime_error("input vector size must be 4!");
    return {v[0], v[1], v[2], v[3]};
}

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
    using cmodel_status_cp  = cmodel_status_t::ConstSharedPtr;
    using cmodel_command_t  = aist_robotiq_msgs::msg::CModelCommand;
    using joint_state_t	    = sensor_msgs::msg::JointState;
    using gripper_command_t = control_msgs::action::GripperCommand;
    using set_velocity_t    = aist_robotiq_msgs::srv::SetVelocity;
    using goal_uuid_t	    = rclcpp_action::GoalUUID;
    using goal_response_t   = rclcpp_action::GoalResponse;
    using cancel_response_t = rclcpp_action::CancelResponse;
    using callback_group_p  = rclcpp::CallbackGroup::SharedPtr;

    template <class MSG>
    using pub_p		= typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using sub_p		= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using srv_p		= typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_cp	= typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p		= typename SRV::Response::SharedPtr;
    template <class ACT>
    using action_p	= typename rclcpp_action::Server<ACT>::SharedPtr;
    template <class ACT>
    using goal_cp	= std::shared_ptr<const typename ACT::Goal>;
    template <class ACT>
    using goal_handle_t	= rclcpp_action::ServerGoalHandle<ACT>;
    template <class ACT>
    using goal_handle_p	= std::shared_ptr<goal_handle_t<ACT> >;

    using vector_t      = std::vector<double>;
    using array4d_t    = std::array<double, 4>;
    using array4i_t    = std::array<int,    4>;

  public:
		CModelController(const rclcpp::NodeOptions& options)	;

  private:
    void	set_velocity_cb(req_cp<set_velocity_t> req,
				res_p<set_velocity_t>  res)		;
    goal_response_t
		goal_cb(const goal_uuid_t&,
			goal_cp<gripper_command_t> goal)		;
    cancel_response_t
		cancel_cb(goal_handle_p<gripper_command_t>)		;
    void	handle_accepted_cb(
		    goal_handle_p<gripper_command_t> goal_handle)	;
    void	cmodel_status_cb(const cmodel_status_cp& status)	;

    void	calibrate()						;
    array4i_t	send_move_command(const array4d_t& position,
                                  const array4d_t& velocity,
				  const array4d_t& max_effort)	const	;
    void	send_raw_move_command(const array4i_t& pos,
                                      const array4i_t& vel,
                                      const array4i_t& eff)     const	;

    vector_t	actual_position(const cmodel_status_cp& status)	const	;
    vector_t	actual_effort(const cmodel_status_cp& status)	const	;
    u_int 	error(const cmodel_status_cp& status)		const	;
    bool	stalled(const cmodel_status_cp& status)		const	;
    bool	reached_goal(const cmodel_status_cp& status)	const	;
    bool	is_active(const cmodel_status_cp& status)	const	;
    bool	is_moving(const cmodel_status_cp& status)	const	;
    double	position_per_tick(size_t i)			const	;
    double	velocity_per_tick(size_t i)			const	;
    double	effort_per_tick(size_t i)			const	;

  private:
  // Read-only parameters
    const int                           _slave_id;
    const array4d_t			_min_position;
    const array4d_t			_max_position;
    const array4d_t			_min_velocity;
    const array4d_t			_max_velocity;
    const array4d_t			_min_effort;
    const array4d_t			_max_effort;

  // Position parameters to be calibrated
    array4i_t                   	_min_gap_counts;
    array4i_t                           _max_gap_counts;
    int					_calibration_step;

  // Publisher for JointState
    joint_state_t                       _joint_state;
    const pub_p<joint_state_t>		_joint_state_pub;

  // Service for setting velocity
    double				_velocity;
    const srv_p<set_velocity_t>		_set_velocity_srv;

  // Publisher for command to the driver
    const pub_p<cmodel_command_t>	_cmodel_command_pub;
    int					_goal_r_pr;

  // Subscriber for Status from the driver
    cmodel_status_cp			_cmodel_status;
    const callback_group_p		_cmodel_status_cbg;
    const sub_p<cmodel_status_t>	_cmodel_status_sub;

  // Gripper command action stuffs
    const action_p<gripper_command_t>	_gripper_command_srv;
    goal_handle_p<gripper_command_t>	_current_goal_handle;
    std::mutex				_current_goal_mtx;
};

CModelController::CModelController(const rclcpp::NodeOptions& options)
    :rclcpp::Node("cmodel_controller", options),
     _slave_id(ddynamic_reconfigure2::declare_read_only_parameter(
                   this, "slave_id", 9)),
     _min_position(array4_from_vector(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "min_position",
                           vector_t{1.047, 1.047, 1.047, 0.160}))),
     _max_position(array4_from_vector(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "max_position",
                           vector_t{0.000, 0.000, 0.000, -0.250}))),
     _min_velocity(array4_from_vector(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "min_velocity",
                           vector_t{0.020, 0.020, 0.020, 0.020}))),
     _max_velocity(array4_from_vector(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "max_velocity",
                           vector_t{0.110, 0.110, 0.110, 0.110}))),
     _min_effort(array4_from_vector(
                     ddynamic_reconfigure2::declare_read_only_parameter(
                         this, "min_effort",
                         vector_t{40.0, 40.0, 40.0, 40.0}))),
     _max_effort(array4_from_vector(
                     ddynamic_reconfigure2::declare_read_only_parameter(
                         this, "max_effort",
                         vector_t{185.0, 185.0, 185.0, 185.0}))),

     _min_gap_counts{255, 255, 255, 255},
     _max_gap_counts{0, 0, 0, 0},
     _calibration_step(0),

     _joint_state(),
     _joint_state_pub(create_publisher<joint_state_t>("/joint_states", 1)),

     _velocity(0.5*(_min_velocity[0] + _max_velocity[0])),
     _set_velocity_srv(create_service<set_velocity_t>(
			   "~/set_velocity",
			   std::bind(&CModelController::set_velocity_cb,
				     this,
				     std::placeholders::_1,
				     std::placeholders::_2))),

     _cmodel_command_pub(create_publisher<cmodel_command_t>("/command", 1)),
     _goal_r_pr(0),

     _cmodel_status(nullptr),
     _cmodel_status_cbg(create_callback_group(
			    rclcpp::CallbackGroupType::MutuallyExclusive)),
     _cmodel_status_sub(create_subscription<cmodel_status_t>(
			    "/status", 1,
			    std::bind(&CModelController::cmodel_status_cb,
				      this, std::placeholders::_1),
			    create_subscription_options(_cmodel_status_cbg))),

     _gripper_command_srv(rclcpp_action::create_server<gripper_command_t>(
			      this, "~/gripper_cmd",
			      std::bind(&CModelController::goal_cb, this,
					std::placeholders::_1,
					std::placeholders::_2),
			      std::bind(&CModelController::cancel_cb, this,
					std::placeholders::_1),
			      std::bind(&CModelController::
					handle_accepted_cb, this,
					std::placeholders::_1))),
     _current_goal_handle(nullptr),
     _current_goal_mtx()
{
    using namespace	std::chrono_literals;

    _joint_state.name = ddynamic_reconfigure2::declare_read_only_parameter(
                            this, "joints",
                            std::vector<std::string>{"finger_joint"});
    if (_joint_state.name.size() != 4)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "4 joint names must be specified!");
        throw;
    }
    _joint_state.position.resize(_joint_state.name.size(), 0.0);
    _joint_state.velocity.resize(_joint_state.name.size(), 0.0);
    _joint_state.effort  .resize(_joint_state.name.size(), 0.0);
    _joint_state.header.stamp.sec     = 0;
    _joint_state.header.stamp.nanosec = 0;

    rclcpp::sleep_for(2s);	// wait for server comes up
    calibrate();

    RCLCPP_INFO_STREAM(get_logger(), "controller started");
}

void
CModelController::set_velocity_cb(req_cp<set_velocity_t> req,
				  res_p<set_velocity_t>  res)
{
    _velocity = req->velocity;
    res->success = true;
}

CModelController::goal_response_t
CModelController::goal_cb(const goal_uuid_t&, goal_cp<gripper_command_t> goal)
{
    RCLCPP_INFO_STREAM(get_logger(),
		       "goal ACCEPTED: position=" << goal->command.position
		       << ", max_effort=" << goal->command.max_effort);
    return goal_response_t::ACCEPT_AND_EXECUTE;
}

CModelController::cancel_response_t
CModelController::cancel_cb(goal_handle_p<gripper_command_t>)
{
    RCLCPP_DEBUG_STREAM(get_logger(), "accepted request for cancelling goal");
    return cancel_response_t::ACCEPT;
}

void
CModelController::handle_accepted_cb(goal_handle_p<gripper_command_t> goal_handle)
{
    const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

  // If any active goal exists, abort it.
    if (_current_goal_handle != nullptr && _current_goal_handle->is_active())
    {
	auto	result = std::make_unique<gripper_command_t::Result>();
	result->position     = actual_position(_cmodel_status)[0];
	result->effort	     = actual_effort(_cmodel_status)[0];
	result->stalled	     = stalled(_cmodel_status);
	result->reached_goal = reached_goal(_cmodel_status);
	_current_goal_handle->abort(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_WARN_STREAM(get_logger(), "previous goal ABORTED");
    }
    _current_goal_handle = goal_handle;

  // Send a move command to the gripper.
    _goal_r_pr = send_move_command(
		     goal_handle->get_goal()->command.position, _velocity,
		     goal_handle->get_goal()->command.max_effort);
}

void
CModelController::cmodel_status_cb(const cmodel_status_cp& status)
{
    using namespace	std::chrono_literals;

  // Reject if slave ID of the input status is not a one of this
    if (status->g_sid != _slave_id)
        return;

  // Keep the latest status for aborting previous goal.
    _cmodel_status = status;

  // Handle calibration process if not moving.
    if (is_active(status) && !is_moving(status))
    {
        if (_calibration_step == 1)
        {
            RCLCPP_INFO_STREAM(get_logger(),
        		       "calibration step 1: start calibration");
            _calibration_step = 2;
            send_raw_move_command({0, 0, 0, 0},
                                  {64, 64, 64, 64},
                                  {1, 1, 1, 1});	// full-open
            rclcpp::sleep_for(3s);
        }
        else if (_calibration_step == 2)
        {
            _max_gap_counts[0] = status->g_po;	// record at full-open
            _max_gap_counts[1] = status->g_pob;	// record at full-open
            _max_gap_counts[2] = status->g_poc;	// record at full-open
            _max_gap_counts[3] = status->g_pos;	// record at full-open
            RCLCPP_INFO_STREAM(get_logger(), "calibration step 2: gap["
        		       << _max_gap_counts[0] << ','
        		       << _max_gap_counts[1] << ','
        		       << _max_gap_counts[2] << ','
        		       << _max_gap_counts[3] << "]@full-open");
            _calibration_step = 3;
            send_raw_move_command({255, 255, 255, 255},
                                  {64, 64, 64, 64},
                                  {1, 1, 1, 1});	// full-close
            rclcpp::sleep_for(3s);
        }
        else if (_calibration_step == 3)
        {
            _min_gap_counts[0] = status->g_po;	// record at full-close
            _min_gap_counts[1] = status->g_pob;	// record at full-close
            _min_gap_counts[2] = status->g_poc;	// record at full-close
            _min_gap_counts[3] = status->g_pos;	// record at full-close
            RCLCPP_INFO_STREAM(get_logger(), "calibration step 3: gap["
        		       << _min_gap_counts[0] << ','
        		       << _min_gap_counts[1] << ','
        		       << _min_gap_counts[2] << ','
        		       << _min_gap_counts[3] << "]@full-close");
            _calibration_step = 0;
            send_raw_move_command({0, 0, 0, 0},
                                  {64, 64, 64, 64},
                                  {1, 1, 1, 1});	// full-open
            RCLCPP_INFO_STREAM(get_logger(), "calibrated to ["
        		       << _min_gap_counts[0] << ','
        		       << _max_gap_counts[0] << "], ["
        		       << _min_gap_counts[1] << ','
        		       << _max_gap_counts[1] << "], ["
        		       << _min_gap_counts[2] << ','
        		       << _max_gap_counts[2] << "], ["
        		       << _min_gap_counts[3] << ','
        		       << _max_gap_counts[3] << ']');
        }
    }

    if (_calibration_step != 0)
        return;

  // Publish joint states of the gripper.
    _joint_state.header.stamp = now();
    _joint_state.position     = actual_position(status);
    _joint_state.effort       = actual_effort(status);
    _joint_state_pub->publish(_joint_state);

  // Check if the current goal is active.
    if (!_current_goal_handle || !_current_goal_handle->is_active())
	return;

    const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

    auto	result = std::make_unique<gripper_command_t::Result>();
    result->position     = actual_position(status)[0];
    result->effort	 = actual_effort(status)[0];
    result->stalled	 = stalled(status);
    result->reached_goal = reached_goal(status);

    if (error(status))	// Check if any error occured in the driver.
    {
	_current_goal_handle->abort(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_ERROR_STREAM(get_logger(), "goal ABORTED[error code:"
			    << error(status) << ']');
	return;
    }
    else if (_current_goal_handle->is_canceling())
    {
	_current_goal_handle->canceled(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_WARN_STREAM(get_logger(), "goal CANCELED");
	return;
    }
    else if (result->reached_goal)
    {
	_current_goal_handle->succeed(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_INFO_STREAM(get_logger(), "goal SUCCEEDED[reached goal]");
	return;
    }
    else if (result->stalled)
    {
	_current_goal_handle->succeed(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_INFO_STREAM(get_logger(), "goal SUCCEEDED[stalled]");
	return;
    }

  // Publish speed and filtered current as a feedback.
    auto	feedback = std::make_unique<gripper_command_t::Feedback>();
    feedback->position	   = result->position;
    feedback->effort	   = result->effort;
    feedback->stalled	   = result->stalled;
    feedback->reached_goal = result->reached_goal;
    _current_goal_handle->publish_feedback(std::move(feedback));
}

void
CModelController::calibrate()
{
    _calibration_step = 1;
}

CModelController::array4i_t
CModelController::send_move_command(const array4d_t& position,
				    const array4d_t& velocity,
                                    const array4d_t& effort) const
{
    array4i_t   pos, vel, eff;
    for (size_t i = 0; i < 4; ++i)
    {
        pos[i] = std::clamp(int((position[i] - _min_position[i])
                                / position_per_tick(i))
                            + _min_gap_counts[i],
                            _max_gap_counts[i], _min_gap_counts[i]);
        vel[i] = std::clamp(int((velocity[i] - _min_velocity[i])
                                / velocity_per_tick(i)),
                            0, 255);
        eff[i] = std::clamp(int((effort[i] - _min_effort[i])
                                / effort_per_tick(i)),
                            0, 255);
    }
    send_raw_move_command(pos, vel, eff);
    return pos;
}

void
CModelController::send_raw_move_command(const array4i_t& pos,
                                        const array4i_t& vel,
                                        const array4i_t& eff) const
{
    auto	cmodel_command = std::make_unique<cmodel_command_t>();
    cmodel_command->r_sid = _slave_id;
    cmodel_command->r_act = 1;
    cmodel_command->r_gto = 1;
    cmodel_command->r_pr  = pos[0];
    cmodel_command->r_sp  = vel[0];
    cmodel_command->r_fr  = eff[0];
    cmodel_command->r_prb = pos[1];
    cmodel_command->r_spb = vel[1];
    cmodel_command->r_frb = eff[1];
    cmodel_command->r_prc = pos[2];
    cmodel_command->r_spc = vel[2];
    cmodel_command->r_frc = eff[2];
    cmodel_command->r_prs = pos[3];
    cmodel_command->r_sps = vel[3];
    cmodel_command->r_frs = eff[3];
    _cmodel_command_pub->publish(std::move(cmodel_command));
}

CModelController::vector_t
CModelController::actual_position(const cmodel_status_cp& status) const
{
    vector_t    position(_joint_state.name.size());
    position[0] = (status->g_po  - _min_gap_counts[0]) * position_per_tick(0)
                + _min_position[0];
    position[1] = (status->g_pob - _min_gap_counts[1]) * position_per_tick(1)
                + _min_position[1];
    position[2] = (status->g_poc - _min_gap_counts[2]) * position_per_tick(2)
                + _min_position[2];
    position[3] = (status->g_pos - _min_gap_counts[3]) * position_per_tick(3)
                + _min_position[3];
    return position;
}

CModelController::vector_t
CModelController::actual_effort(const cmodel_status_cp& status) const
{
    vector_t    effort(_joint_state.name.size());
    effort[0] = status->g_cou * effort_per_tick(0) + _min_effort[0];
    effort[1] = status->g_cub * effort_per_tick(1) + _min_effort[1];
    effort[2] = status->g_cuc * effort_per_tick(2) + _min_effort[2];
    effort[3] = status->g_cus * effort_per_tick(3) + _min_effort[3];

    return effort;
}

bool
CModelController::stalled(const cmodel_status_cp& status) const
{
  // After the goal accepted in _goal_cb(), status->g_pr does not
  // correctly reflects the requested position if cmodel_status_cb() is
  // called before send_move_command(). Thus we have to use _goal_r_pr
  // instead of status->g_pr.
    return (status->g_obj == 1 && status->g_po > _goal_r_pr + 1) ||
	   (status->g_obj == 2 && status->g_po + 1 < _goal_r_pr);
}

bool
CModelController::reached_goal(const cmodel_status_cp& status) const
{
    return status->g_obj == 3
        && abs(status->g_po - _goal_r_pr) <= 1
        && abs(status->g_po - _goal_r_pr) <= 1
        && abs(status->g_po - _goal_r_pr) <= 1
        && abs(status->g_po - _goal_r_pr) <= 1;
}

u_int
CModelController::error(const cmodel_status_cp& status) const
{
    return status->g_flt;
}

bool
CModelController::is_active(const cmodel_status_cp& status) const
{
    return status->g_sta == 3 && status->g_act == 1;
}

bool
CModelController::is_moving(const cmodel_status_cp& status) const
{
    return status->g_gto == 1 && status->g_obj == 0;
}

double
CModelController::position_per_tick(size_t i) const
{
    return (_max_position[i]   - _min_position[i])
         / (_max_gap_counts[i] - _min_gap_counts[i]);
}

double
CModelController::velocity_per_tick(size_t i) const
{
    return (_max_velocity[i] - _min_velocity[i])/255;
}

double
CModelController::effort_per_tick(size_t i) const
{
    return (_max_effort[i] - _min_effort[i])/255;
}
}	// namespace aist_robotiq

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_robotiq::CModelController)
