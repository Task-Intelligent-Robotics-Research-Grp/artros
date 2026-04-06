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
 *  \file       cmodel_controller_base.hpp
 *  \brief	base class of controllers for the Robotiq hands
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <aist_robotiq_msgs/msg/c_model_status.hpp>
#include <aist_robotiq_msgs/msg/c_model_command.hpp>
#include <aist_robotiq_msgs/srv/set_velocity.hpp>
#include <aist_robotiq_msgs/action/gripper3_f_command.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include <Eigen/Core>

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
*  class CModelController<DOF>                                          *
************************************************************************/
template <size_t DOF>
class CModelController : public rclcpp::Node
{
  private:
    using dvector_t     = std::vector<double>;
    using darray_t      = Eigen::Array<double, DOF, 1>;
    using iarray_t      = Eigen::Array<int,    DOF, 1>;
    using dof_t         = std::integral_constant<size_t, DOF>;
    using one_t         = std::integral_constant<size_t, 1>;
    using four_t        = std::integral_constant<size_t, 4>;

    using cmodel_status_t       = aist_robotiq_msgs::msg::CModelStatus;
    using cmodel_status_cp      = cmodel_status_t::ConstSharedPtr;
    using cmodel_command_t      = aist_robotiq_msgs::msg::CModelCommand;
    using joint_state_t         = sensor_msgs::msg::JointState;
    using gripper_command_t     = control_msgs::action::GripperCommand;
    using gripper3f_command_t   = aist_robotiq_msgs::action::Gripper3FCommand;
    using set_velocity_t        = aist_robotiq_msgs::srv::SetVelocity;
    using goal_uuid_t           = rclcpp_action::GoalUUID;
    using goal_response_t       = rclcpp_action::GoalResponse;
    using cancel_response_t     = rclcpp_action::CancelResponse;
    using callback_group_p      = rclcpp::CallbackGroup::SharedPtr;
    using action_t              = std::conditional_t<DOF == 1,
                                                     gripper_command_t,
                                                     gripper3f_command_t>;

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
    using action_p      = typename rclcpp_action::Server<ACT>::SharedPtr;
    template <class ACT>
    using goal_cp       = std::shared_ptr<const typename ACT::Goal>;
    template <class ACT>
    using goal_handle_t = typename rclcpp_action::ServerGoalHandle<ACT>;
    template <class ACT>
    using goal_handle_p = std::shared_ptr<goal_handle_t<ACT> >;
    template <class ACT>
    using result_p      = std::unique_ptr<typename ACT::Result>;
    template <class ACT>
    using feedback_p    = std::unique_ptr<typename ACT::Feedback>;


  public:
		CModelController(const rclcpp::NodeOptions& options)	;

  private:
    void	set_velocity_cb(req_cp<set_velocity_t> req,
				res_p<set_velocity_t>  res)		;
    goal_response_t
		goal_cb(const goal_uuid_t&, goal_cp<action_t> goal)	;
    cancel_response_t
		cancel_cb(goal_handle_p<action_t>)                      ;
    void	handle_accepted_cb(goal_handle_p<action_t> goal_handle)	;
    void	cmodel_status_cb(const cmodel_status_cp& status)	;

    static darray_t
                desired_position(const goal_cp<gripper_command_t>& goal)
                {
                    return darray_t{goal->command.position};
                }
    static darray_t
                desired_effort(const goal_cp<gripper_command_t>& goal)
                {
                    return darray_t{goal->command.max_effort};
                }
    void        set_result(const result_p<gripper_command_t>& result,
                           const cmodel_status_cp& status) const
                {
                    result->position     = actual_position(status)[0];
                    result->effort	 = actual_effort(status)[0];
                    result->stalled	 = stalled(status);
                    result->reached_goal = reached_goal(status);
                }
    static darray_t
                desired_position(const goal_cp<gripper3f_command_t>& goal)
                {
                    return darray_t{goal->gap};
                }
    static darray_t
                desired_effort(const goal_cp<gripper3f_command_t>& goal)
                {
                    return darray_t{goal->max_effort};
                }
    void        set_result(const result_p<gripper3f_command_t>& result,
                           const cmodel_status_cp& status) const
                {
                    result->position     = actual_position(status)[0];
                    result->effort	 = actual_effort(status)[0];
                    result->stalled	 = stalled(status);
                    result->reached_goal = reached_goal(status);
                }

    void	calibrate()
                {
                    _calibration_step = 1;
                }
    iarray_t	send_move_command(const darray_t& position,
                                  const darray_t& velocity,
				  const darray_t& max_effort) const
                {
                    const auto  pos = clamp(((position - _min_position)
                                             / position_per_tick())
                                            .template cast<int>()
                                            + _min_gap_counts,
                                            _max_gap_counts, _min_gap_counts);
                    const auto  vel = clamp(((velocity - _min_velocity)
                                             / velocity_per_tick())
                                            .template cast<int>(),
                                            iarray_t{0}, iarray_t{255});
                    const auto  eff = clamp(((max_effort - _min_effort)
                                             / effort_per_tick())
                                            .template cast<int>(),
                                            iarray_t{0}, iarray_t{255});
                    send_raw_move_command(pos, vel, eff, dof_t());
                    return pos;
                }
    void	send_raw_move_command(const iarray_t& pos, const iarray_t& vel,
                                      const iarray_t& eff, one_t) const
                {
                    auto cmodel_command = std::make_unique<cmodel_command_t>();
                    cmodel_command->r_sid = _slave_id;
                    cmodel_command->r_act = 1;
                    cmodel_command->r_gto = 1;
                    cmodel_command->r_pr  = pos[0];
                    cmodel_command->r_sp  = vel[0];
                    cmodel_command->r_fr  = eff[0];
                    _cmodel_command_pub->publish(std::move(cmodel_command));
                }
    void        send_raw_move_command(const iarray_t& pos, const iarray_t& vel,
                                      const iarray_t& eff, four_t) const
                {
                    auto cmodel_command = std::make_unique<cmodel_command_t>();
                    cmodel_command->r_sid = _slave_id;
                    cmodel_command->r_act = 1;
                    cmodel_command->r_gto = 1;
                    cmodel_command->r_icf = 1;
                    cmodel_command->r_ics = 1;
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

    darray_t	actual_position(const cmodel_status_cp& status) const
                {
                    return (pos(status) - _min_gap_counts)
                          .template cast<double>() * position_per_tick()
                        + _min_position;
                }
    darray_t	actual_effort(const cmodel_status_cp& status) const
                {
                    return eff(status).template cast<double>()
                         * effort_per_tick() + _min_effort;
                }
    static iarray_t
                pos(const cmodel_status_cp& status)
                {
                    return pos(status, dof_t());
                }
    static iarray_t
                pos(const cmodel_status_cp& status, one_t)
                {
                    return iarray_t{status->g_po};
                }
    static iarray_t
                pos(const cmodel_status_cp& status, four_t)
                {
                    return iarray_t{status->g_po,  status->g_pob,
                                    status->g_poc, status->g_pos};
                }
    static iarray_t
                eff(const cmodel_status_cp& status)
                {
                    return eff(status, dof_t());
                }
    static iarray_t
                eff(const cmodel_status_cp& status, one_t)
                {
                    return iarray_t{status->g_cou};
                }
    static iarray_t
                eff(const cmodel_status_cp& status, four_t)
                {
                    return iarray_t{status->g_cou, status->g_cub,
                                    status->g_cuc, status->g_cus};
                }
    static u_int
                error(const cmodel_status_cp& status)
                {
                    return status->g_flt;
                }
    bool	stalled(const cmodel_status_cp& status) const
                {
                  // After the goal accepted in _goal_cb(), status->g_pr
                  // does not correctly reflects the requested position if
                  // cmodel_status_cb() is called before send_move_command().
                  // Thus we have to use _goal_r_pr instead of status->g_pr.
                    return (status->g_obj == 1 &&
                            (pos(status) > _goal_r_pr + 1).all()) ||
                           (status->g_obj == 2 &&
                            (pos(status) < _goal_r_pr - 1).all());
                }
    bool	reached_goal(const cmodel_status_cp& status) const
                {
                    return status->g_obj == 3 &&
                           (abs(pos(status) - _goal_r_pr) <= 1).all();
                }
    static bool	is_active(const cmodel_status_cp& status)
                {
                    return status->g_sta == 3 && status->g_act == 1;
                }
    static bool	is_moving(const cmodel_status_cp& status)
                {
                    return status->g_gto == 1 && status->g_obj == 0;
                }

    darray_t	position_per_tick() const
                {
                    return (_max_position - _min_position)
                         / (_max_gap_counts - _min_gap_counts)
                        .template cast<double>();
                }
    darray_t	velocity_per_tick() const
                {
                    return (_max_velocity - _min_velocity) / 255.0;
                }
    darray_t	effort_per_tick() const
                {
                    return (_max_effort - _min_effort) / 255.0;
                }

    static iarray_t
                clamp(const iarray_t& x,
                      const iarray_t& min, const iarray_t& max)
                {
                    iarray_t    val;
                    for (ssize_t i = 0; i < x.size(); ++i)
                        val[i] = std::clamp(x[i], min[i], max[i]);
                    return val;
                }
    static darray_t
                dvector_to_darray(const dvector_t& v)
                {
                    darray_t    a;
                    if (ssize_t(v.size()) < a.size())
                        throw std::runtime_error(
                                  "input vector size must be at least "
                                  + std::to_string(a.size()) + '!');
                    for (ssize_t i = 0; i < a.size(); ++i)
                        a[i] = v[i];
                    return a;
                }
    static dvector_t
                darray_to_dvector(const darray_t& a)
                {
                    dvector_t   v(a.size());
                    for (size_t i = 0; i < v.size(); ++i)
                        v[i] = a[i];
                    return v;
                }

  private:
  // Read-only parameters
    const int                           _slave_id;
    const darray_t			_min_position;
    const darray_t			_max_position;
    const darray_t			_min_velocity;
    const darray_t			_max_velocity;
    const darray_t			_min_effort;
    const darray_t			_max_effort;

  // Position parameters to be calibrated
    iarray_t                       	_min_gap_counts;
    iarray_t                            _max_gap_counts;
    int					_calibration_step;

  // Publisher for JointState
    joint_state_t                       _joint_state;
    const pub_p<joint_state_t>		_joint_state_pub;

  // Service for setting velocity
    double				_velocity;
    const srv_p<set_velocity_t>		_set_velocity_srv;

  // Publisher for command to the driver
    const pub_p<cmodel_command_t>	_cmodel_command_pub;
    iarray_t				_goal_r_pr;

  // Subscriber for Status from the driver
    cmodel_status_cp			_cmodel_status;
    const callback_group_p		_cmodel_status_cbg;
    const sub_p<cmodel_status_t>	_cmodel_status_sub;

  // Gripper command action stuffs
    const action_p<action_t>	_gripper_command_srv;
    goal_handle_p<action_t>	_current_goal_handle;
    std::mutex				_current_goal_mtx;
};

template <size_t DOF>
CModelController<DOF>::CModelController(const rclcpp::NodeOptions& options)
    :rclcpp::Node("cmodel_controller", options),
     _slave_id(ddynamic_reconfigure2::declare_read_only_parameter(
                   this, "slave_id", 9)),
     _min_position(dvector_to_darray(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "min_position",
                           dvector_t{1.047, 1.047, 1.047, 0.160}))),
     _max_position(dvector_to_darray(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "max_position",
                           dvector_t{0.000, 0.000, 0.000, -0.250}))),
     _min_velocity(dvector_to_darray(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "min_velocity",
                           dvector_t{0.020, 0.020, 0.020, 0.020}))),
     _max_velocity(dvector_to_darray(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "max_velocity",
                           dvector_t{0.110, 0.110, 0.110, 0.110}))),
     _min_effort(dvector_to_darray(
                     ddynamic_reconfigure2::declare_read_only_parameter(
                         this, "min_effort",
                         dvector_t{40.0, 40.0, 40.0, 40.0}))),
     _max_effort(dvector_to_darray(
                     ddynamic_reconfigure2::declare_read_only_parameter(
                         this, "max_effort",
                         dvector_t{185.0, 185.0, 185.0, 185.0}))),

     _min_gap_counts{255},
     _max_gap_counts{0},
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

     _gripper_command_srv(rclcpp_action::create_server<action_t>(
			      this, "~/gripper_cmd",
			      std::bind(&CModelController::goal_cb, this,
					std::placeholders::_1,
					std::placeholders::_2),
			      std::bind(&CModelController::cancel_cb,
                                        this, std::placeholders::_1),
			      std::bind(&CModelController::handle_accepted_cb,
                                        this, std::placeholders::_1))),
     _current_goal_handle(nullptr),
     _current_goal_mtx()
{
    using namespace	std::chrono_literals;

    _joint_state.name = ddynamic_reconfigure2::declare_read_only_parameter(
                            this, "joints",
                            std::vector<std::string>{"finger_joint"});
    if (_joint_state.name.size() != DOF)
    {
        RCLCPP_ERROR_STREAM(get_logger(),
                            DOF << " joint names must be specified!");
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

template <size_t DOF> void
CModelController<DOF>::set_velocity_cb(req_cp<set_velocity_t> req,
                                       res_p<set_velocity_t>  res)
{
    _velocity = req->velocity;
    res->success = true;
}

template <size_t DOF> typename CModelController<DOF>::goal_response_t
CModelController<DOF>::goal_cb(const goal_uuid_t&, goal_cp<action_t>)
{
    RCLCPP_INFO_STREAM(get_logger(), "goal ACCEPTED");
    return goal_response_t::ACCEPT_AND_EXECUTE;
}

template <size_t DOF> typename CModelController<DOF>::cancel_response_t
CModelController<DOF>::cancel_cb(goal_handle_p<action_t>)
{
    RCLCPP_DEBUG_STREAM(get_logger(), "accepted request for cancelling goal");
    return cancel_response_t::ACCEPT;
}

template <size_t DOF> void
CModelController<DOF>::handle_accepted_cb(goal_handle_p<action_t> goal_handle)
{
    const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

  // If any active goal exists, abort it.
    if (_current_goal_handle != nullptr && _current_goal_handle->is_active())
    {
	auto	result = std::make_unique<typename action_t::Result>();
        set_result(result, _cmodel_status);
	_current_goal_handle->abort(std::move(result));
	_current_goal_handle = nullptr;

	RCLCPP_WARN_STREAM(get_logger(), "previous goal ABORTED");
    }
    _current_goal_handle = goal_handle;

  // Send a move command to the gripper.
    _goal_r_pr = send_move_command(desired_position(goal_handle->get_goal()),
                                   darray_t{_velocity},
                                   desired_effort(goal_handle->get_goal()));
}

template <size_t DOF> void
CModelController<DOF>::cmodel_status_cb(const cmodel_status_cp& status)
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
            send_raw_move_command(iarray_t{0}, iarray_t{64}, iarray_t{1},
                                  dof_t());             // full-open
            rclcpp::sleep_for(3s);
        }
        else if (_calibration_step == 2)
        {
            _max_gap_counts = pos(status);              // record at full-open
            RCLCPP_INFO_STREAM(get_logger(), "calibration step 2: gap["
        		       << _max_gap_counts.transpose()
                               << "]@full-open");
            _calibration_step = 3;
            send_raw_move_command(iarray_t{255}, iarray_t{64}, iarray_t{1},
                                  dof_t());             // full-close
            rclcpp::sleep_for(3s);
        }
        else if (_calibration_step == 3)
        {
            _min_gap_counts = pos(status);              // record at full-close
            RCLCPP_INFO_STREAM(get_logger(), "calibration step 3: gap["
        		       << _min_gap_counts.transpose()
                               << "]@full-close");
            _calibration_step = 0;
            send_raw_move_command(iarray_t{0}, iarray_t{64}, iarray_t{1},
                                  dof_t());             // full-open
            RCLCPP_INFO_STREAM(get_logger(), "calibrated to [("
        		       << _min_gap_counts.transpose() << "), ("
        		       << _max_gap_counts.transpose() << ")]");
        }
    }

    if (_calibration_step != 0)
        return;

  // Publish joint states of the gripper.
    _joint_state.header.stamp = now();
    _joint_state.position     = darray_to_dvector(actual_position(status));
    _joint_state.effort       = darray_to_dvector(actual_effort(status));
    _joint_state_pub->publish(_joint_state);

  // Check if the current goal is active.
    if (!_current_goal_handle || !_current_goal_handle->is_active())
	return;

    const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

    auto	result = std::make_unique<typename action_t::Result>();
    set_result(result, status);

  // Publish speed and filtered current as a feedback.
    auto	feedback = std::make_unique<typename action_t::Feedback>();
    feedback->position	   = result->position;
    feedback->effort	   = result->effort;
    feedback->stalled	   = result->stalled;
    feedback->reached_goal = result->reached_goal;
    _current_goal_handle->publish_feedback(std::move(feedback));

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
}
}	// namespace aist_robotiq
