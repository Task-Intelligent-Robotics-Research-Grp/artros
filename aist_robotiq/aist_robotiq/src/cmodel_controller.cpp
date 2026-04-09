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
 *  \file       cmodel_controller.cpp
 *  \brief      controller for Robotiq grippers
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <aist_robotiq_msgs/msg/c_model_status.hpp>
#include <aist_robotiq_msgs/msg/c_model_command.hpp>
#include <aist_robotiq_msgs/srv/set_velocity.hpp>
#include <aist_robotiq_msgs/srv/set_mode.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include <Eigen/Core>

namespace aist_robotiq
{
/************************************************************************
*  static functions                                                     *
************************************************************************/
static inline rclcpp::SubscriptionOptions
create_subscription_options(const rclcpp::CallbackGroup::SharedPtr& cbg)
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = cbg;
    return options;
}

/************************************************************************
*  class CModelController                                               *
************************************************************************/
class CModelController : public rclcpp::Node
{
  private:
    using vector_t      = std::vector<double>;
    using array4d       = Eigen::Array4d;
    using array4i       = Eigen::Array4i;

    using cmodel_status_t       = aist_robotiq_msgs::msg::CModelStatus;
    using cmodel_status_cp      = cmodel_status_t::ConstSharedPtr;
    using cmodel_command_t      = aist_robotiq_msgs::msg::CModelCommand;
    using joint_state_t         = sensor_msgs::msg::JointState;
    using gripper_command_t     = control_msgs::action::GripperCommand;
    using set_velocity_t        = aist_robotiq_msgs::srv::SetVelocity;
    using set_mode_t            = aist_robotiq_msgs::srv::SetMode;
    using goal_uuid_t           = rclcpp_action::GoalUUID;
    using goal_response_t       = rclcpp_action::GoalResponse;
    using cancel_response_t     = rclcpp_action::CancelResponse;
    using callback_group_p      = rclcpp::CallbackGroup::SharedPtr;

    template <class MSG>
    using pub_p         = typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using sub_p         = typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using srv_p         = typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_cp        = typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p         = typename SRV::Response::SharedPtr;
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
                CModelController(const rclcpp::NodeOptions& options)    ;

  private:
    ssize_t     dof() const
                {
                    return _joint_state.name.size();
                }
    void        trigger_calibration()
                {
                    _calibration_step = 1;
                }
    void        do_calibration(const cmodel_status_cp& status)          ;

    void        set_velocity_cb(req_cp<set_velocity_t> req,
                                res_p<set_velocity_t>  res)
                {
                    _velocity = req->velocity;
                    res->success = true;
                }
    void        set_mode_cb(req_cp<set_mode_t> req, res_p<set_mode_t> res)
                {
                    set_mode(req->mode);
                    res->success = true;
                }

    goal_response_t
                goal_cb(const goal_uuid_t&,
                        goal_cp<gripper_command_t> goal)                ;
    cancel_response_t
                cancel_cb(goal_handle_p<gripper_command_t>)             ;
    void        handle_accepted_cb(
                    goal_handle_p<gripper_command_t> goal_handle)       ;
    void        cmodel_status_cb(const cmodel_status_cp& status)        ;
    void        set_mode(u_int mode)
                {
                    _mode = mode;

                    if (_mode >= set_mode_t::Request::ICF)
                        return;

                    auto cmodel_command = std::make_unique<cmodel_command_t>();
                    cmodel_command->r_sid = _slave_id;
                    cmodel_command->r_act = 1;
                    cmodel_command->r_mod = _mode;

                    _cmodel_command_pub->publish(std::move(cmodel_command));
                }

    array4d     desired_position(const goal_cp<gripper_command_t>& goal) const
                {
                    return array4d{goal->command.position};
                }
    array4d     desired_velocity() const
                {
                    return array4d{_velocity};
                }
    array4d     desired_effort(const goal_cp<gripper_command_t>& goal)
                {
                    return array4d{goal->command.max_effort};
                }
    void        set_result(const result_p<gripper_command_t>& result,
                           const cmodel_status_cp& status) const
                {
                    result->position     = actual_position(status)[0];
                    result->effort       = actual_effort(status)[0];
                    result->stalled      = stalled(status);
                    result->reached_goal = reached_goal(status);
                }

    array4i     send_move_command(const array4d& position,
                                  const array4d& velocity,
                                  const array4d& effort) const
                {
                    const auto  pos = clamp(((position - _min_position) /
                                             position_per_tick()).cast<int>()
                                            + _min_pos,
                                            _max_pos, _min_pos);
                    const auto  vel = clamp(((velocity - _min_velocity) /
                                             velocity_per_tick()).cast<int>(),
                                            array4i{0}, array4i{255});
                    const auto  eff = clamp(((effort - _min_effort) /
                                             effort_per_tick()).cast<int>(),
                                            array4i{0}, array4i{255});
                    send_raw_move_command(pos, vel, eff);
                    return pos;
                }
    void        send_raw_move_command(const array4i& pos,
                                      const array4i& vel,
                                      const array4i& eff) const
                {
                    RCLCPP_WARN_STREAM(get_logger(), "pos=("
                                       << pos.transpose() << ')');

                    auto cmodel_command = std::make_unique<cmodel_command_t>();
                    cmodel_command->r_sid = _slave_id;
                    cmodel_command->r_act = 1;
                    switch (_mode)
                    {
                      case set_mode_t::Request::ICF:
                        cmodel_command->r_icf = 1;
                        cmodel_command->r_ics = 0;
                        break;
                      case set_mode_t::Request::ICS:
                        cmodel_command->r_icf = 0;
                        cmodel_command->r_ics = 1;
                        break;
                      default:
                        cmodel_command->r_mod = _mode;
                        cmodel_command->r_icf = 0;
                        cmodel_command->r_ics = 0;
                        break;
                    }
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

    array4d     actual_position(const cmodel_status_cp& status) const
                {
                    return (pos(status) - _min_pos).cast<double>()
                         * position_per_tick() + _min_position;
                }
    array4d     actual_effort(const cmodel_status_cp& status) const
                {
                    return eff(status).cast<double>() * effort_per_tick()
                         + _min_effort;
                }
    static array4i
                pos(const cmodel_status_cp& status)
                {
                    return array4i{status->g_po,  status->g_pob,
                                   status->g_poc, status->g_pos};
                }
    static array4i
                eff(const cmodel_status_cp& status)
                {
                    return array4i{status->g_cou, status->g_cub,
                                   status->g_cuc, status->g_cus};
                }
    static u_int
                error(const cmodel_status_cp& status)
                {
                    return status->g_flt;
                }
    bool        stalled(const cmodel_status_cp& status) const
                {
                    const auto  p = pos(status);

                  // After the goal accepted in _goal_cb(), status->g_pr
                  // does not correctly reflects the requested position if
                  // cmodel_status_cb() is called before send_move_command().
                  // Thus we have to use _goal_r_pr instead of status->g_pr.
                    switch (_mode)
                    {
                      default:
                        break;
                      case set_mode_t::Request::ICF:
                        return (status->g_obj == 1       &&
                                p[0] > _goal_r_pr[0] + 1 &&
                                p[1] > _goal_r_pr[1] + 1 &&
                                p[2] > _goal_r_pr[2] + 1) ||
                               (status->g_obj == 2 &&
                                p[0] < _goal_r_pr[0] - 1 &&
                                p[1] < _goal_r_pr[1] - 1 &&
                                p[2] < _goal_r_pr[2] - 1);
                      case set_mode_t::Request::ICS:
                        return (status->g_obj == 1       &&
                                p[3] > _goal_r_pr[3] + 1) ||
                               (status->g_obj == 2       &&
                                p[3] < _goal_r_pr[3] - 1);
                    }
                    return (status->g_obj == 1       &&
                            p[0] > _goal_r_pr[0] + 1) ||
                           (status->g_obj == 2       &&
                            p[0] < _goal_r_pr[0] - 1);
                }
    bool        reached_goal(const cmodel_status_cp& status) const
                {
                    return status->g_obj == 3;
                    // return status->g_obj == 3 &&
                    //        (abs(pos(status) - _goal_r_pr) <= 1).all();
                }
    static bool is_active(const cmodel_status_cp& status)
                {
                    return status->g_act == 1 && status->g_sta == 3;
                }
    static bool is_moving(const cmodel_status_cp& status)
                {
                    return status->g_gto == 1 && status->g_obj == 0;
                }

    array4d     position_per_tick() const
                {
                    return (_max_position - _min_position)
                         / (_max_pos - _min_pos).cast<double>();
                }
    array4d     velocity_per_tick() const
                {
                    return (_max_velocity - _min_velocity) / 255.0;
                }
    array4d     effort_per_tick() const
                {
                    return (_max_effort - _min_effort) / 255.0;
                }

    static array4i
                clamp(const array4i& x,
                      const array4i& min, const array4i& max)
                {
                    return array4i{std::clamp(x[0], min[0], max[0]),
                                   std::clamp(x[1], min[1], max[1]),
                                   std::clamp(x[2], min[2], max[2]),
                                   std::clamp(x[3], min[3], max[3])};
                }
    static array4d
                vector_to_array4d(const vector_t& v)
                {
                    array4d    a;
                    for (ssize_t i = 0; i < a.size(); ++i)
                        a[i] = (size_t(i) < v.size() ? v[i] : a[0]);
                    return a;
                }

  private:
  // Read-only parameters
    const int                           _slave_id;
    const array4d                       _min_gap;
    const array4d                       _max_gap;
    const array4d                       _min_position;
    const array4d                       _max_position;
    const array4d                       _min_velocity;
    const array4d                       _max_velocity;
    const array4d                       _min_effort;
    const array4d                       _max_effort;

  // Position parameters to be calibrated
    array4i                             _min_pos;
    array4i                             _max_pos;
    int                                 _calibration_step;

  // Publisher for JointState
    joint_state_t                       _joint_state;
    const pub_p<joint_state_t>          _joint_state_pub;

  // Service for setting velocity
    double                              _velocity;
    const srv_p<set_velocity_t>         _set_velocity_srv;

  // Service for setting mode
    u_int                               _mode;
    const srv_p<set_mode_t>             _set_mode_srv;

  // Publisher for command to the driver
    const pub_p<cmodel_command_t>       _cmodel_command_pub;
    array4i                             _goal_r_pr;

  // Subscriber for Status from the driver
    cmodel_status_cp                    _cmodel_status;
    const callback_group_p              _cmodel_status_cbg;
    const sub_p<cmodel_status_t>        _cmodel_status_sub;

  // Gripper command action stuffs
    const action_p<gripper_command_t>   _gripper_command_srv;
    goal_handle_p<gripper_command_t>    _current_goal_handle;
    std::mutex                          _current_goal_mtx;
};

CModelController::CModelController(const rclcpp::NodeOptions& options)
    :rclcpp::Node("cmodel_controller", options),
     _slave_id(ddynamic_reconfigure2::declare_read_only_parameter(
                   this, "slave_id", 9)),
     _min_gap(vector_to_array4d(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "min_gap", vector_t{0.0}))),
     _max_gap(vector_to_array4d(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "max_gap", vector_t{0.085}))),
     _min_position(vector_to_array4d(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "min_position", vector_t{0.81}))),
     _max_position(vector_to_array4d(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "max_position", vector_t{0.00}))),
     _min_velocity(vector_to_array4d(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "min_velocity", vector_t{0.020}))),
     _max_velocity(vector_to_array4d(
                       ddynamic_reconfigure2::declare_read_only_parameter(
                           this, "max_velocity", vector_t{0.150}))),
     _min_effort(vector_to_array4d(
                     ddynamic_reconfigure2::declare_read_only_parameter(
                         this, "min_effort", vector_t{0.0}))),
     _max_effort(vector_to_array4d(
                     ddynamic_reconfigure2::declare_read_only_parameter(
                         this, "max_effort", vector_t{235.0}))),

     _min_pos(),
     _max_pos(),
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

     _mode(set_mode_t::Request::BASIC),
     _set_mode_srv(_min_position.size() == 4 ?
                   create_service<set_mode_t>(
                       "~/set_mode",
                       std::bind(&CModelController::set_mode_cb,
                                 this,
                                 std::placeholders::_1,
                                 std::placeholders::_2)) :
                   nullptr),

     _cmodel_command_pub(create_publisher<cmodel_command_t>("/command", 1)),
     _goal_r_pr{0},

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
                              std::bind(&CModelController::cancel_cb,
                                        this, std::placeholders::_1),
                              std::bind(&CModelController::handle_accepted_cb,
                                        this, std::placeholders::_1))),
     _current_goal_handle(nullptr),
     _current_goal_mtx()
{
    using namespace     std::chrono_literals;

    _joint_state.name = ddynamic_reconfigure2::declare_read_only_parameter(
                            this, "joints",
                            std::vector<std::string>{"finger_joint"});
    if (dof() != 1 && dof() != 4)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "The number of joints["
                            << dof() << "] must be one or four!");
        throw;
    }

    _joint_state.position.resize(dof(), 0.0);
    _joint_state.velocity.resize(dof(), 0.0);
    _joint_state.effort  .resize(dof(), 0.0);
    _joint_state.header.stamp.sec     = 0;
    _joint_state.header.stamp.nanosec = 0;

    rclcpp::sleep_for(2s);      // wait for server comes up
    trigger_calibration();

    RCLCPP_INFO_STREAM(get_logger(), "controller started");
}

void
CModelController::do_calibration(const cmodel_status_cp& status)
{
    using namespace     std::chrono_literals;

    switch (_calibration_step)
    {
      default:
        return;
      case 1:
        RCLCPP_INFO_STREAM(get_logger(),
                           "calibration step 1: start finger calibration");
        send_raw_move_command(array4i{0}, array4i{64}, array4i{1});    // open
        ++_calibration_step;
        break;
      case 2:
        _max_pos = pos(status);             // record at full-open
        RCLCPP_INFO_STREAM(get_logger(), "calibration step 2: finger pos["
                           << _max_pos.transpose() << "]@full-open");
        send_raw_move_command(array4i{255}, array4i{64}, array4i{1});  // close
        ++_calibration_step;
        break;
      case 3:
        _min_pos = pos(status);             // record at full-close
        RCLCPP_INFO_STREAM(get_logger(), "calibration step 3: finger pos["
                           << _min_pos.transpose() << "]@full-close");
        send_raw_move_command(array4i{0}, array4i{64}, array4i{1});    // open
        if (dof() == 1)
            _calibration_step = 8;
        else
            ++_calibration_step;
        break;
      case 4:
        set_mode(set_mode_t::Request::SCISSOR); // switch to scissor mode
        RCLCPP_INFO_STREAM(get_logger(),
                           "calibration step 4: switch to scissor mode");
        ++_calibration_step;
        break;
      case 5:
        RCLCPP_INFO_STREAM(get_logger(),
                           "calibration step 5: start scissor calibration");
        send_raw_move_command(array4i{0}, array4i{64}, array4i{1});    // open
        ++_calibration_step;
        break;
      case 6:
        _max_pos[3] = pos(status)[3];       // record at full-open
        RCLCPP_INFO_STREAM(get_logger(), "calibration step 6: scissor pos["
                           << _max_pos[3] << "]@full-open");
        send_raw_move_command(array4i{255}, array4i{64}, array4i{1});  // close
        ++_calibration_step;
        break;
      case 7:
        _min_pos[3] = pos(status)[3];       // record at full-close
        RCLCPP_INFO_STREAM(get_logger(), "calibration step 7: sissor pos["
                           << _min_pos[3] << "]@full-close");
        set_mode(set_mode_t::Request::BASIC);   // switch back to basic mode
        ++_calibration_step;
        break;
      case 8:
        RCLCPP_INFO_STREAM(get_logger(), "calibration completed: range[("
                           << _min_pos.transpose() << ")-("
                           << _max_pos.transpose() << ")]");
        _calibration_step = 0;
        break;
    }

    rclcpp::sleep_for(500ms);
}

CModelController::goal_response_t
CModelController::goal_cb(const goal_uuid_t&, goal_cp<gripper_command_t> goal)
{
    if (_calibration_step)
    {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "goal REJECTED: calibration not completed");
        return goal_response_t::REJECT;
    }
    RCLCPP_INFO_STREAM(get_logger(), "goal ACCEPTED: position="
                       << goal->command.position);
    return goal_response_t::ACCEPT_AND_EXECUTE;
}

CModelController::cancel_response_t
CModelController::cancel_cb(goal_handle_p<gripper_command_t>)
{
    RCLCPP_DEBUG_STREAM(get_logger(), "request for cancelling goal accepted");
    return cancel_response_t::ACCEPT;
}

void
CModelController::handle_accepted_cb(
    goal_handle_p<gripper_command_t> goal_handle)
{
    const std::lock_guard<std::mutex>   lock(_current_goal_mtx);

  // If any active goal exists, abort it.
    if (_current_goal_handle != nullptr && _current_goal_handle->is_active())
    {
        auto    result = std::make_unique<gripper_command_t::Result>();
        set_result(result, _cmodel_status);
        _current_goal_handle->abort(std::move(result));
        _current_goal_handle = nullptr;

        RCLCPP_WARN_STREAM(get_logger(), "previous goal ABORTED");
    }
    _current_goal_handle = goal_handle;

  // Send a move command to the gripper.
    _goal_r_pr = send_move_command(desired_position(goal_handle->get_goal()),
                                   desired_velocity(),
                                   desired_effort(goal_handle->get_goal()));
}

void
CModelController::cmodel_status_cb(const cmodel_status_cp& status)
{
  // Reject the input status not of mine.
    if (status->g_sid != _slave_id)
        return;

  // Return immediately if activation or mode swithcing is in progress.
    if (!is_active(status))
        return;

  // Return immediately if any error occured in the driver.
    if (error(status))
    {
        RCLCPP_ERROR_STREAM(get_logger(), "status error[error code:"
                            << error(status) << ']');
        return;
    }

  // Handle calibration process if not moving.
    if (_calibration_step)
    {
        if (!is_moving(status))
            do_calibration(status);
        return;
    }

  // Publish joint states of the gripper.
    const auto  position = actual_position(status);
    const auto  effort   = actual_effort(status);
    _joint_state.header.stamp = now();
    for (ssize_t i = 0; i < dof(); ++i)
    {
        _joint_state.position[i] = position[i];
        _joint_state.effort[i]   = effort[i];
    }
    _joint_state_pub->publish(_joint_state);

  // Check if the current goal is active.
    if (!_current_goal_handle || !_current_goal_handle->is_active())
        return;

  // Process the current goal.
    const std::lock_guard<std::mutex>   lock(_current_goal_mtx);

    _cmodel_status = status;  // Keep the latest status for aborting the goal.

    auto        result = std::make_unique<gripper_command_t::Result>();
    set_result(result, status);

    if (error(status))  // Check if any error occured in the driver.
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
    auto        feedback = std::make_unique<gripper_command_t::Feedback>();
    feedback->position     = result->position;
    feedback->effort       = result->effort;
    feedback->stalled      = result->stalled;
    feedback->reached_goal = result->reached_goal;
    _current_goal_handle->publish_feedback(std::move(feedback));
}
}       // namespace aist_robotiq

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_robotiq::CModelController)
