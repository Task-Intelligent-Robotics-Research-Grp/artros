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
/*!
 *  \file	butterworth.hpp
 *  \author	Toshio Ueshiba
 *  \brief	Butterworth low-pass filter of even order
 */
#include <filters/filter_base.hpp>
#include <aist_utility/butterworth_lpf.hpp>
#include <Eigen/Core>

namespace aist_filters
{
/**********************************************************************
*  class ButterworthFilter<T>                                         *
**********************************************************************/
template <class T>
class ButterworthFilter : public filters::FilterBase<T>
{
  private:
    using super = filters::FilterBase<T>;
    using lpf_t = aist_utility::ButterworthLPF<T>;

  public:
                ButterworthFilter()                                     {}
                ~ButterworthFilter()                                    {}

    size_t	half_order()            const   { return _lpf.half_order(); }
    T           cutoff()                const   { return _lpf.cutoff(); }
    bool        configure()                                     override;
    bool        update(const T& data_in, T& data_out)           override;

  private:
    lpf_t       _lpf;
};

template <class T> bool
ButterworthFilter<T>::configure()
{
    size_t      half_order;
    if (!super::getParam("cutoff", half_order))
    {
        RCLCPP_ERROR_STREAM(this->logging_interface_->get_logger(),
                            "ButterworthFilter did not find param half_order");
        return false;
    }

    double      cutoff;
    if (!super::getParam("cutoff", cutoff))
    {
        RCLCPP_ERROR_STREAM(this->logging_interface_->get_logger(),
                            "ButterworthFilter did not find param cutoff");
        return false;
    }

    _lpf.initialize(half_order, T(cutoff));
    return true;
}

template <class T> bool
ButterworthFilter<T>::update(const T& data_in, T& data_out)
{
    data_out = _lpf.filter(data_in);
    return true;
}

/**********************************************************************
*  class MultiChannelButterworthFilter<T>                             *
**********************************************************************/
template <class T>
class MultiChannelButterworthFilter : public filters::MultiChannelFilterBase<T>
{
  private:
    using super   = filters::MultiChannelFilterBase<T>;
    using value_t = Eigen::VectorX<T>;
    using lpf_t   = aist_utility::ButterworthLPF<T, value_t>;

  public:
                MultiChannelButterworthFilter()                         {}
                ~MultiChannelButterworthFilter()                        {}

    size_t	half_order()            const   { return _lpf.half_order(); }
    T           cutoff()                const   { return _lpf.cutoff(); }
    bool        configure()                                     override;
    bool        update(const std::vector<T> & data_in,
                       std::vector<T> & data_out)               override;

  private:
    lpf_t       _lpf;
};

template <class T> bool
MultiChannelButterworthFilter<T>::configure()
{
    size_t      half_order;
    if (!super::getParam("half_order", half_order))
    {
        RCLCPP_ERROR_STREAM(this->logging_interface_->get_logger(),
                            "ButterworthFilter did not find param half_order");
        return false;
    }

    double      cutoff;
    if (!super::getParam("cutoff", cutoff))
    {
        RCLCPP_ERROR_STREAM(this->logging_interface_->get_logger(),
                            "ButterworthFilter did not find param cutoff");
        return false;
    }

    _lpf.initialize(half_order, T(cutoff));
    return true;
}

template <class T> bool
MultiChannelButterworthFilter<T>::update(const std::vector<T>& data_in,
                                         std::vector<T>& data_out)
{
    using cmap = Eigen::Map<const value_t>;
    using map  = Eigen::Map<value_t>;

    if (data_in.size()  != this->number_of_channels_ ||
        data_out.size() != this->number_of_channels_)
    {
        RCLCPP_ERROR(
            this->logging_interface_->get_logger(),
            "Configured with wrong size config: %ld, in: %ld out: %ld",
            this->number_of_channels_, data_in.size(), data_out.size());
        return false;
    }

    map(data_out.data(), 1) = _lpf.filter(cmap(data_in.data(), 1));
    return true;
}
}  // namespace filters
