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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <filters/filter_base.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <aist_utility/butterworth_lpf.hpp>
#include <aist_filters/butterworth_filter_parameters.hpp>

namespace aist_filters
{
template <class T>
class ButterworthFilter : public filters::FilterBase<T>
{
  private:
    template <class T_>
    static T_           _element(T_)                                    ;
    template <class T_>
    static T_           _element(std::vector<T_>)                       ;
    static double       _element(geometry_msgs::msg::WrenchStamped)     ;

    using element_t	= decltype(_element(std::declval<T>()));
    using traits_t      = control_toolbox::FilterTraits<T>;
    using storage_t     = typename traits_t::StorageType;
    using lpf_t         = aist_utility::butterworth_lpf<element_t, storage_t>;

  public:
    bool        configure()                                     override;
    bool        update(const T& data_in, T& data_out)           override;

  private:
    std::shared_ptr<rclcpp::Logger>                     logger_;
    std::shared_ptr<low_pass_filter::ParamListener>     parameter_handler_;
    low_pass_filter::Params                             parameters_;
    std::shared_ptr<lpf_t> lpf_;
};

template <class T> bool
ButterworthFilter<T>::configure()
{
    logger_.reset(new rclcpp::Logger(
                      this->logging_interface_->get_logger().get_child(
                          this->filter_name_)));

  // Initialize the parameters once
    if (!parameter_handler_)
    {
        try
        {
            parameter_handler_ = std::make_shared<low_pass_filter::ParamListener>(
                this->params_interface_, this->param_prefix_);
        }
        catch (const std::exception & ex)
        {
            RCLCPP_ERROR((*logger_),
                         "Butterworth filter cannot be configured: %s (type : %s)", ex.what(),
                typeid(ex).name());
            parameter_handler_.reset();
            return false;
        }
        catch (...)
        {
            RCLCPP_ERROR((*logger_), "Caught unknown exception while configuring Butterworth filter");
            parameter_handler_.reset();
            return false;
        }
    }
    parameters_ = parameter_handler_->get_params();
    lpf_ = std::make_shared<lpf_t>(
        parameters_.sampling_frequency, parameters_.damping_frequency, parameters_.damping_intensity);

    return lpf_->configure();
}

template <class T> bool
ButterworthFilter<T>::update(const T& data_in, T& data_out)
{
    if (!this->configured_ || !lpf_ || !lpf_->is_configured())
    {
        throw std::runtime_error("Filter is not configured");
    }

  // Update internal parameters if required
    if (parameter_handler_->is_old(parameters_))
    {
        parameters_ = parameter_handler_->get_params();
        lpf_->set_params(
            parameters_.sampling_frequency, parameters_.damping_frequency, parameters_.damping_intensity);
    }

    return lpf_->update(data_in, data_out);
}

}  // namespace aist_filters
