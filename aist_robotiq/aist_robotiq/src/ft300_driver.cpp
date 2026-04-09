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
 *  \file       ft300_driver.cpp
 *  \brief      ROS driver for Robotiq FT300 force-torque sensors
 */
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>         // for struct sockaddr_in
#include <arpa/inet.h>          // for inet_addr()
#include <netdb.h>              // for struct hostent, gethostbyname()
#include <errno.h>

namespace aist_robotiq
{
/************************************************************************
*  static functions                                                     *
************************************************************************/
static const char*
splitd(const char* s, double& val)
{
    for (; *s; ++s)
        if (*s == '+' || *s == '-' || *s == '.' || isdigit(*s))
        {
            char*       end;
            val = strtod(s, &end);
            return end;
        }

    throw std::runtime_error("No strings representing numeric values found.");
    return nullptr;
}

template <class T> static T     stov(const std::string& s);
template <> std::string
stov<std::string>(const std::string& s) { return s; }
template <> int
stov<int>(const std::string& s)         { return std::stoi(s); }
template <> double
stov<double>(const std::string& s)      { return std::stod(s); }

/************************************************************************
*  class ft300_driver                                                   *
************************************************************************/
class ft300_driver : public hardware_interface::SensorInterface
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ft300_driver)

  private:
    using super         = hardware_interface::SensorInterface;
    using return_type   = hardware_interface::return_type;
    using timer_p       = rclcpp::TimerBase::SharedPtr;

  public:
                        ft300_driver()                                  ;

    CallbackReturn      on_configure(const rclcpp_lifecycle::State&)    ;
    CallbackReturn      on_cleanup(const rclcpp_lifecycle::State&)      ;
    return_type         read(const rclcpp::Time&,
                             const rclcpp::Duration&)                   ;

  private:
    template <class T>
    T                   get_param(const std::string& name, T default_value)
                        {
                            const auto  s = info_.hardware_parameters[name];
                            return (s != "" ? stov<T>(s): default_value);
                        }
    void                tick()                                          ;
    bool                up_socket()                                     ;
    bool                connect_socket(u_long s_addr, int port)         ;

  private:
    int                 _socket;
    double              _ft[6];

    timer_p             _timer;
    rclcpp::Logger      _logger;
};

ft300_driver::ft300_driver()
    :_socket(-1),
     _ft{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
     _timer(create_wall_timer(std::chrono::duration<double>(
                                  1.0/get_param<double>("rate", 125.0)),
                              std::bind(&ft300_driver::tick, this))),
     _logger(rclcpp::get_logger("ft300_driver"))
{
}

hardware_interface::CallbackReturn
ft300_driver::on_configure(const rclcpp_lifecycle::State&)
{
    _socket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR_STREAM(_logger,
                            "failed to open socket: " << strerror(errno));
        return CallbackReturn::ERROR;
    }

    if (!up_socket())
    {
        RCLCPP_ERROR_STREAM(_logger,
                            "failed to bringup socket: " << strerror(errno));
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO_STREAM(_logger, "ft300_driver initialized.");

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ft300_driver::on_cleanup(const rclcpp_lifecycle::State&)
{
    if (_socket >= 0)
        ::close(_socket);
}

hardware_interface::return_type
ft300_driver::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    std::array<char, 1024>      buf;
    const auto                  nbytes = ::read(_socket,
                                                buf.data(), buf.size());
    if (nbytes < 0)
    {
        RCLCPP_ERROR_STREAM(_logger,
                            "(ft300_driver) failed to read from socket: "
                            << strerror(errno));
        return return_type::ERROR;
    }
    buf[nbytes] = '\0';

    const char* s = buf.data();
    s = splitd(s, _ft[0]);
    s = splitd(s, _ft[1]);
    s = splitd(s, _ft[2]);
    s = splitd(s, _ft[3]);
    s = splitd(s, _ft[4]);
    s = splitd(s, _ft[5]);

    return return_type::OK;
}

void
ft300_driver::tick()
{
    read(get_clock()->now(), rate.cycleTime());
    manager.update(ros::Time::now(), rate.cycleTime());
}

bool
ft300_driver::up_socket()
{
  // Get hoastname and port from parameters.
    const auto  hostname = get_param<std::string>("hostname", "192.168.1.1");
    const auto  port     = get_param<int>("port", 63351);

  // Connect socket to hostname:port.
    auto        addr = inet_addr(hostname.c_str());
    if (addr != 0xffffffff)
        return connect_socket(addr, port);

    const auto  h = gethostbyname(hostname.c_str());
    if (!h)
    {
        RCLCPP_ERROR_STREAM(_logger,
                            "(ft300_driver) unknown host name: " << hostname);
        return false;
    }

    for (auto addr_ptr = (u_long**)h->h_addr_list; *addr_ptr; ++addr_ptr)
        if (connect_socket(*(*addr_ptr), port))
            return true;

    return false;
}

bool
ft300_driver::connect_socket(u_long s_addr, int port)
{
    sockaddr_in server;
    server.sin_family      = AF_INET;
    server.sin_port        = htons(port);
    server.sin_addr.s_addr = s_addr;
    RCLCPP_INFO_STREAM(_logger, "trying to connect socket to "
                       << inet_ntoa(server.sin_addr) << ':' << port << "...");
    if (::connect(_socket, (sockaddr*)&server, sizeof(server)) == 0)
    {
        RCLCPP_INFO_STREAM(_logger, "succeeded.");
        return true;
    }
    else
    {
        RCLCPP_ERROR_STREAM(_logger, "failed: " << strerror(errno));
        return false;
    }
}

}       // namepsace aist_robotiq

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_robotiq::ft300_driver)
