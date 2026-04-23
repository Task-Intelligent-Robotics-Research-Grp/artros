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
#include <aist_robotiq_msgs/msg/c_model_status.hpp>
#include <aist_robotiq_msgs/msg/c_model_command.hpp>
#include <aist_robotiq_msgs/msg/object_state.hpp>
#include <aist_robotiq_msgs/action/suction_command.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

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
*  class SuctionController                                              *
************************************************************************/
class SuctionController : public rclcpp::Node
{
  private:
    using cmodel_status_t       = aist_robotiq_msgs::msg::CModelStatus;
    using cmodel_status_cp      = cmodel_status_t::ConstSharedPtr;
    using cmodel_command_t      = aist_robotiq_msgs::msg::CModelCommand;
    using object_state_t        = aist_robotiq_msgs::msg::ObjectState;
    using suction_command_t     = aist_robotiq_msgs::action::SuctionCommand;
    using goal_uuid_t           = rclcpp_action::GoalUUID;
    using goal_response_t       = rclcpp_action::GoalResponse;
    using cancel_response_t     = rclcpp_action::CancelResponse;
    using callback_group_p      = rclcpp::CallbackGroup::SharedPtr;
    using duration_t            = builtin_interfaces::msg::Duration;
    using array2i               = std::array<int, 2>;

    template <class MSG>
    using pub_p         = typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using sub_p         = typename rclcpp::Subscription<MSG>::SharedPtr;
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
                SuctionController(const rclcpp::NodeOptions& options)   ;

  private:
  // CModelStatus subscription stuffs
    void        cmodel_status_cb(const cmodel_status_cp& status)        ;

  // SuctionCommand action stuffs
    goal_response_t
                goal_cb(const goal_uuid_t&,
                        goal_cp<suction_command_t> goal);
    void        handle_accepted_cb(
                    goal_handle_p<suction_command_t> goal_handle)       ;
    cancel_response_t
                cancel_cb(goal_handle_p<suction_command_t>)             ;
    void        process_suction_command(const cmodel_status_cp& status) ;
    void        set_result(const result_p<suction_command_t>& result,
                           const cmodel_status_cp& status) const
                {
                    result->pressure = actual_pressure(status);
                    result->stalled  = stalled(status);
                    result->fault    = fault(status);
                }

  // Utilities
    void        send_reset_command() const
                {
                    using namespace     std::chrono_literals;

                    cmodel_command_t    cmodel_command;
                    cmodel_command.r_sid = _slave_id;
                    cmodel_command.r_act = 0;
                    cmodel_command.r_gto = 0;
                    _cmodel_command_pub->publish(cmodel_command);
                    rclcpp::sleep_for(100ms);

                    cmodel_command.r_act = 1;
                    cmodel_command.r_gto = 1;
                    _cmodel_command_pub->publish(cmodel_command);
                    rclcpp::sleep_for(100ms);
                }
    array2i     send_suction_command(bool advanced_mode,
                                     double max_pressure, double min_pressure,
                                     const duration_t& timeout) const
                {
                    const auto  max_prs = std::clamp(int(max_pressure + 100.0),
                                                     0, 255);
                    const auto  min_prs = std::clamp(int(min_pressure + 100.0),
                                                     0, 100);
                    const auto  tmo_sec = timeout.sec + timeout.nanosec*1.0e-9;
                    const auto  tmo     = std::clamp(int(tmo_sec), 0, 255);
                    send_raw_suction_command(advanced_mode,
                                             max_prs, min_prs, tmo);
                    return {max_prs, min_prs};
                }
    void        send_raw_suction_command(bool advanced_mode, int max_prs,
                                         int min_prs, int tmo) const
                {
                    auto cmodel_command = std::make_unique<cmodel_command_t>();
                    cmodel_command->r_sid = _slave_id;
                    cmodel_command->r_act = 1;
                    cmodel_command->r_mod = (advanced_mode ? 1 : 0);
                    cmodel_command->r_gto = 1;
                    cmodel_command->r_atr = 0;
                    cmodel_command->r_pr  = max_prs;
                    cmodel_command->r_sp  = tmo;
                    cmodel_command->r_fr  = min_prs;

                    _cmodel_command_pub->publish(std::move(cmodel_command));
                }
    void        send_stop_command() const
                {
                    auto cmodel_command = std::make_unique<cmodel_command_t>();
                    cmodel_command->r_sid = _slave_id;
                    cmodel_command->r_act = 1;
                    cmodel_command->r_gto = 0;

                    _cmodel_command_pub->publish(std::move(cmodel_command));
                }

    static double
                actual_pressure(const cmodel_status_cp& status)
                {
                    return status->g_po - 100.0;
                }
    static bool is_active(const cmodel_status_cp& status)
                {
                    return status->g_act == 1 && status->g_sta == 3;
                }
    static u_int
                fault(const cmodel_status_cp& status)
                {
                    return status->g_flt;
                }
    bool        stalled(const cmodel_status_cp& status) const
                {
                    return _goal_pr[0] < _goal_pr[1] &&
                           status->g_pr == _goal_pr[0] &&
                           (status->g_obj == 1 || status->g_obj == 2);
                }
    bool        released(const cmodel_status_cp& status) const
                {
                    return _goal_pr[0] > _goal_pr[1] &&
                           status->g_pr == _goal_pr[0] &&
                           (status->g_obj == 0 || status->g_obj == 3);
                }

  private:
  // Read-only parameters
    const int                           _slave_id;

  // Publisher for command to the driver
    const pub_p<cmodel_command_t>       _cmodel_command_pub;

  // Subscriber for Status from the driver
    cmodel_status_cp                    _cmodel_status;
    const callback_group_p              _cmodel_status_cbg;
    const sub_p<cmodel_status_t>        _cmodel_status_sub;

  // Publisher for SuctionState
    const pub_p<object_state_t>         _object_state_pub;

  // SuctionCommand action stuffs
    array2i                             _goal_pr;
    const action_p<suction_command_t>   _suction_command_srv;
    goal_handle_p<suction_command_t>    _goal_handle;
    std::mutex                          _goal_mtx;
};

SuctionController::SuctionController(const rclcpp::NodeOptions& options)
    :rclcpp::Node("suction_controller", options),
     _slave_id(ddynamic_reconfigure2::declare_read_only_parameter(
                   this, "slave_id", 9)),

     _cmodel_command_pub(create_publisher<cmodel_command_t>("/command", 1)),

     _cmodel_status(nullptr),
     _cmodel_status_cbg(create_callback_group(
                            rclcpp::CallbackGroupType::MutuallyExclusive)),
     _cmodel_status_sub(create_subscription<cmodel_status_t>(
                            "/status", 1,
                            std::bind(&SuctionController::cmodel_status_cb,
                                      this, std::placeholders::_1),
                            create_subscription_options(_cmodel_status_cbg))),

     _object_state_pub(create_publisher<object_state_t>("~/object_state", 1)),

     _goal_pr{0, 0},
     _suction_command_srv(rclcpp_action::create_server<suction_command_t>(
                              this, "~/gripper_cmd",
                              std::bind(&SuctionController::goal_cb, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2),
                              std::bind(&SuctionController::cancel_cb, this,
                                        std::placeholders::_1),
                              std::bind(&SuctionController::
                                        handle_accepted_cb, this,
                                        std::placeholders::_1))),
     _goal_handle(nullptr),
     _goal_mtx()
{
    RCLCPP_INFO_STREAM(get_logger(), "suction controller started");
}

/*
 *  CModelStatus subscription stuffs
 */
void
SuctionController::cmodel_status_cb(const cmodel_status_cp& status)
{
  // Reject the input status not of mine.
    if (status->g_sid != _slave_id)
        return;

  // Publish state of object detection.
    auto        object_state = std::make_unique<object_state_t>();
    object_state->state = status->g_obj;
    _object_state_pub->publish(std::move(object_state));

  // Return immediately if activation is in progress.
    if (!is_active(status))
        return;

    const std::lock_guard<std::mutex>   lock(_goal_mtx);

  // If the goal of SuctionCommand is active, process it.
    if (_goal_handle && _goal_handle->is_active())
        process_suction_command(status);
}

/*
 *  SuctionCommand action stuffs
 */
SuctionController::goal_response_t
SuctionController::goal_cb(const goal_uuid_t&, goal_cp<suction_command_t> goal)
{
    RCLCPP_INFO_STREAM(get_logger(),
                       "SuctionCommand goal ACCEPTED[advanced_mode="
                       << std::boolalpha << goal->command.advanced_mode
                       << ", max_pressure=" << goal->command.max_pressure
                       << ", min_pressure=" << goal->command.min_pressure
                       << ']');
    return goal_response_t::ACCEPT_AND_EXECUTE;
}

void
SuctionController::handle_accepted_cb(
    goal_handle_p<suction_command_t> goal_handle)
{
    const std::lock_guard<std::mutex>   lock(_goal_mtx);

  // If any active goal exists, abort it.
    if (_goal_handle != nullptr && _goal_handle->is_active())
    {
        RCLCPP_WARN_STREAM(get_logger(),
                           "previous SuctionCommand goal ABORTED");

        auto    result = std::make_unique<suction_command_t::Result>();
        set_result(result, _cmodel_status);
        _goal_handle->abort(std::move(result));
        _goal_handle = nullptr;
    }
    _goal_handle = goal_handle;

  // Send a move command to the suction.
    const auto& command = goal_handle->get_goal()->command;
    _goal_pr = send_suction_command(command.advanced_mode,
                                    command.max_pressure, command.min_pressure,
                                    command.timeout);
}

SuctionController::cancel_response_t
SuctionController::cancel_cb(goal_handle_p<suction_command_t>)
{
    RCLCPP_DEBUG_STREAM(get_logger(),
                        "request for cancelling SuctionCommand goal accepted");
    return cancel_response_t::ACCEPT;
}

void
SuctionController::process_suction_command(const cmodel_status_cp& status)
{
    _cmodel_status = status;  // Keep the latest status for aborting the goal.

    auto        result = std::make_unique<suction_command_t::Result>();
    set_result(result, status);

    if (const auto fault_code=fault(status))
    {
        std::string     fault_message;

        switch (fault(status))
        {
          case suction_command_t::Result::ACTION_DELAYED:
            fault_message = "action delayed";
            break;
          case suction_command_t::Result::POROUS_MATERIAL:
            fault_message = "very porous material detected";
            break;
          case suction_command_t::Result::GRIPPING_TIMEOUT:
            fault_message = "gripping timeout";
            break;
          case suction_command_t::Result::TEMPERATURE:
            fault_message = "maximum temperature";
            break;
          case suction_command_t::Result::NO_COMMUNICATION:
            fault_message = "no communication with the gripper during 1 second";
            break;
          default:
            fault_message = "fault_code=" + std::to_string(fault_code);
            break;
        }
        RCLCPP_ERROR_STREAM(get_logger(), "SuctionCommand goal ABORTED["
                            << fault_message << ']');
        _goal_handle->abort(std::move(result));
        _goal_handle = nullptr;
        send_reset_command();
        return;
    }
    else if (_goal_handle->is_canceling())
    {
        RCLCPP_WARN_STREAM(get_logger(), "SuctionCommand goal CANCELED");
        send_stop_command();
        _goal_handle->canceled(std::move(result));
        _goal_handle = nullptr;
        return;
    }
    else if (stalled(status) || released(status))
    {
        RCLCPP_INFO_STREAM(get_logger(),
                           "SuctionCommand goal SUCCEEDED[pressure="
                           << result->pressure
                           << ", stalled=" << std::boolalpha << result->stalled
                           << ']');
        _goal_handle->succeed(std::move(result));
        _goal_handle = nullptr;
        return;
    }

  // Publish speed and filtered current as a feedback.
    auto        feedback = std::make_unique<suction_command_t::Feedback>();
    feedback->pressure = result->pressure;
    feedback->stalled  = result->stalled;
    _goal_handle->publish_feedback(std::move(feedback));
}
}       // namespace aist_robotiq

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_robotiq::SuctionController)
