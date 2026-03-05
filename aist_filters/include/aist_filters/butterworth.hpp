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
 *  \file	butterworth_lpf.h
 *  \author	Toshio Ueshiba
 *  \brief	Butterworth low-pass filter of even order
 */
#pragma once

#include <cmath>
#include <vector>
#include <filters/filter_base.hpp>
#include <control_toolbox/filter_traits.hpp>

namespace aist_filters : public FilterBase<T>
{
/*!
  Butterworth low-pass filter of even order
  \param S	type of coefficients
  \param T	type of signal to be filtered
*/
template <class T>
class ButterworthFilter : public filters::FilterBase<T>
{
  private:
    using super       = filters::FilterBase<T>;
    using Traits      = control_toolbox::FilterTraits<T>;
    using StorageType = typename Traits::StorageType;

    template <class T_>
    static T_           _element(T_)                                    ;
    template <class T_>
    static T_           _element(std::vector<T_>)                       ;
    static double       _element(geometry_msgs::msg::WrenchStamped)     ;

  public:
    using element_type	= decltype(_element(std::declval<T>()));
    using value_type	= T;	//!< type of signal to be filtered

  public:
    explicit	ButterworthFilter()	:_cutoff(0)			{}

  /*!
    バタワースフィルタを生成
    \param half_order	フィルタの次数の半分
    \param cutoff	カットオフ周波数fcをサンプリング周波数fsで正規化した値
			すなわちfc/fs
   */
    explicit	ButterworthFilter() :_configured(false)                 {}

    bool	set_params(size_t half_order, element_type cutoff)
		{
                    constexpr element_type      pi = element_type(M_PI);

		    _cutoff = cutoff;

		    _A.resize(half_order);
		    _d1.resize(_A.size());
		    _d2.resize(_A.size());
		    _y0.resize(_A.size());
		    _y1.resize(_A.size());
		    _y2.resize(_A.size());

		    const auto	a  = std::tan(pi*_cutoff);
		    const auto	a2 = a*a;

		    for (size_t k = 0; k < _A.size(); ++k)
		    {
			const auto	n = _A.size();
			const auto	s = std::sin(pi*element_type(2*k + 1)
                                                     /element_type(4*n));
			const auto	t = 1 + 2*a*s + a2;
			_A[k]  = a2/t;
			_d1[k] = 2*(1 - a2)/t;
			_d2[k] = -(1 - 2*a*s + a2)/t;
		    }
		}

    size_t	half_order() const
		{
		    return _A.size();
		}

    element_type
		cutoff() const
		{
		    return _cutoff;
		}

    bool	update(const T& data_in, T& data_out) const
		{
                    Traits::assign(data_out, data_in);

		  // 2次フィルタをカスケード接続
		    for (size_t k = 0; k < _A.size(); ++k)
		    {
                        Traits::assign(_y0[k],
                                       _d1[k]*_y1[k] + _d2[k]*_y2[k] + data_out);
                        Traits::assign(data_out,
                                       _A[k]*(_y0[k] + 2*_y1[k] + _y2[k]));
                        Traits::assign(_y2[k], _y1[k]);
                        Traits::assign(_y1[k], _y0[k]);
		    }

                    Traits::add_metadata(data_out, data_in);

		    return true;
		}

    void	reset(const value_type& x)
		{
		    for (size_t k = 0; k < _A.size(); ++k)
			_y1[k] = _y2[k] = (1/(1 - _d1[k] - _d2[k]))*x;
		}

  private:
    bool                _configured;
    element_type	_cutoff;
    StorageType		_A;
    StorageType		_d1;
    StorageType		_d2;
    mutable StorageType	_y0;
    mutable StorageType	_y1;
    mutable StorageType	_y2;
};

}  // namespace moveit_servo
