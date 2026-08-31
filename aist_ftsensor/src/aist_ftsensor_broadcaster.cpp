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
 *  \file	aist_ftsensor_controller.cpp
 *  \brief	force-torque sensor controller with gravity compensation
 */
#include <controller_interface/chainable_controller_interface.hpp>
#include <semantic_components/force_torque_sensor.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <cstdlib>		// for std::getenv()
#include <sys/stat.h>		// for mkdir()
#include <aist_utility/eigen.hpp>
#include <aist_utility/butterworth_lpf.hpp>

namespace aist_ftsensor
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T, int M> std::ostream&
operator <<(std::ostream& out, const Eigen::Matrix<T, M, 1>& v)
{
    for (size_t i = 0; i < M; ++i)
	out << ' ' << v(i);
    return out;
}

template <class T> std::ostream&
operator <<(std::ostream& out, const std::vector<T>& v)
{
    for (const auto& elm : v)
	out << ' ' << elm;
    return out << std::endl;
}

inline Eigen::Vector3d
fromKDL(const KDL::Vector& v)
{
    return {v(0), v(1), v(2)};
}

std::ostream&
operator <<(std::ostream& out, const KDL::JntArray& joints)
{
    for (u_int i = 0; i < joints.rows(); ++i)
	out << ' ' << joints(i);
    return out << std::endl;
}

/************************************************************************
*  class ForceTorqueSensorBroadcaster					*
************************************************************************/
class ForceTorqueSensorBroadcaster
    : public controller_interface::ChainableControllerInterface
{
  public:
    using if_config_t	 = controller_interface::InterfaceConfiguration;
    using cb_return_t	 = controller_interface::CallbackReturn;
    using ci_return_t	 = controller_interface::return_type;
    using lc_state_t	 = rclcpp_lifecycle::State;
    using hw_state_if_t	 = hardware_interface::StateInterface;
    
    using wrench_t	 = geometry_msgs::msg::WrenchStamped;
    using joint_state_t  = sensor_msgs::msg::JointState;
    using joint_state_cp = joint_state_t::ConstSharedPtr;

    using fksolver_p	 = std::unique_ptr<KDL::ChainFkSolverPos>;
    using controller_t	 = ForceTorqueSensorBroadcaster;
    using vector_t	 = Eigen::Vector3d;
    using matrix_t	 = Eigen::Matrix3d;
    using quaternion_t	 = Eigen::Quaterniond;
    using ft_t		 = Eigen::Matrix<double, 6, 1>;
    using filter_t	 = aist_utility::ButterworthLPF<double, ft_t>;
    using ddr_t		 = ddynamic_reconfigure2::DDynamicReconfigure<
			       rclcpp_lifecycle::LifecycleNode>;

    using ft_sensor_t	 = semantic_components::ForceTorqueSensor;

    using trigger_t	 = std_srvs::srv::Trigger;
    using trigger_req	 = trigger_t::Request::SharedPtr;
    using trigger_res	 = trigger_t::Response::SharedPtr;

    template <class MSG>
    using publisher_t	 = realtime_tools::RealtimePublisher<MSG>;
    template <class MSG>
    using publisher_p    = std::shared_ptr<publisher_t<MSG> >;
    template <class MSG>
    using subscription_p = typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using service_p	 = typename rclcpp::Service<SRV>::SharedPtr;

    constexpr static double	G = 9.80665;

  public:
		ForceTorqueSensorBroadcaster()				;

    if_config_t	command_interface_configuration()	const	override;
    if_config_t	state_interface_configuration()		const	override;
    
    cb_return_t	on_init()					override;
    cb_return_t	on_configure(const lc_state_t& prev_state)	override;
    cb_return_t	on_activate(const lc_state_t& prev_state)	override;
    cb_return_t	on_deactivate(const lc_state_t& prev_state)	override;

    ci_return_t	update_and_write_commands(
		    const rclcpp::Time& time,
		    const rclcpp::Duration& period)		override;
    ci_return_t	update_reference_from_subscribers(
		    const rclcpp::Time& time,
		    const rclcpp::Duration& period)		override;

    // std::vector<hw_state_if_t>
    // 		on_export_state_interfaces()			override;

  private:
    void	joint_state_cb(const joint_state_cp& joint_state)	;
    void	take_sample_cb(const trigger_req& req,
			       const trigger_res& res)			;
    void	compute_calibration_cb(const trigger_req& req,
				       const trigger_res& res);
    void	save_calibration_cb(const trigger_req& req,
				    const trigger_res& res)		;
    void	clear_samples_cb(const trigger_req& req,
				 const trigger_res& res)		;
    void	reset_bias_cb(const trigger_req& req,
			      const trigger_res& res)			;

    void	take_sample()						;
    bool	compute_calibration()					;
    void	save_calibration(std::ostream& out)		const	;
    void	clear_samples()						;
    void	reset_bias()						;

    void	take_sample(const vector_t& k,
			    const vector_t& f, const vector_t& m)	;
    void	set_filter_half_order(int half_order)			;
    void	set_filter_cutoff_frequency(double cutoff_frequency)	;

    vector_t	vector_param(const std::string& name)			;
    quaternion_t
		quaternion_param(const std::string& name)		;
    KDL::JntArray
		get_jnt_pos(
		    const std::vector<std::string>& jnt_name)	const	;

  private:
  // [A] Wrench stuffs
    std::string				_frame_id;
    std::unique_ptr<ft_sensor_t>	_ft_sensor;
    publisher_p<wrench_t>		_wrench_org_pub;
    publisher_p<wrench_t>		_wrench_pub;
    rclcpp::Duration			_pub_interval;
    rclcpp::Time			_last_pub_time;

  // [B] Filtering stuffs
    filter_t				_filter;
    ddr_t				_ddr;
    ft_t				_ft;
    mutable std::mutex			_ft_mtx;

  // [C] JointState stuffs
    subscription_p<joint_state_t>	_joint_state_sub;
    std::map<std::string, double>	_joint_positions;
    mutable std::mutex			_joint_positions_mtx;

  // [D] Forward kinematics stuffs
    KDL::Chain				_chain;
    std::vector<std::string>		_joint_names;
    fksolver_p				_fksolver;

  // [E] Gravity compensation stuffs
    bool				_compensate_gravity;
    double				_mg;		// effector mass
    quaternion_t			_q;		// rotation
    vector_t				_r;		// mass center
    vector_t				_f0;		// force offset
    vector_t				_m0;		// torque offset

  // [F] Calibration stuffs
    std::string				_calib_file;
    bool				_do_sample;
    bool				_do_reset;
    size_t				_nsamples;
    vector_t				_k_sum;
    vector_t				_f_sum;
    vector_t				_m_sum;
    double				_k_sqsum;
    matrix_t				_kf_sum;
    matrix_t				_km_sum;
    matrix_t				_mm_sum;
    std::ofstream			_fout;
    service_p<trigger_t>		_take_sample;
    service_p<trigger_t>		_compute_calibration;
    service_p<trigger_t>		_save_calibration;
    service_p<trigger_t>		_clear_samples;
    service_p<trigger_t>		_reset_bias;
};

ForceTorqueSensorBroadcaster::ForceTorqueSensorBroadcaster()
    :controller_interface::ChainableControllerInterface(),
     _frame_id(),
     _ft_sensor(),
     _wrench_org_pub(),
     _wrench_pub(),
     _pub_interval(0, 0),
     _last_pub_time(),

     _filter(),
     _ddr(get_node()),
     _ft(ft_t::Zero()),
     _ft_mtx(),

     _joint_state_sub(),
     _joint_positions(),
     _joint_positions_mtx(),

     _chain(),
     _joint_names(),
     _fksolver(),

     _compensate_gravity(false),
     _mg(0.0),
     _q(1.0, 0.0, 0.0, 0.0),
     _r(0.0, 0.0, 0.0),
     _f0(0.0, 0.0, 0.0),
     _m0(0.0, 0.0, 0.0),

     _calib_file(),
     _do_sample(false),
     _do_reset(false),
     _nsamples(0),
     _k_sum(vector_t::Zero()),
     _f_sum(vector_t::Zero()),
     _m_sum(vector_t::Zero()),
     _k_sqsum(0),
     _kf_sum(matrix_t::Zero()),
     _km_sum(matrix_t::Zero()),
     _mm_sum(matrix_t::Zero()),
     _fout(),
     _take_sample(),
     _compute_calibration(),
     _save_calibration(),
     _clear_samples(),
     _reset_bias()
{
}

ForceTorqueSensorBroadcaster::if_config_t
ForceTorqueSensorBroadcaster::command_interface_configuration() const
{
    if_config_t	if_config;
    if_config.type = controller_interface::interface_configuration_type::NONE;
    return if_config;
}

ForceTorqueSensorBroadcaster::if_config_t
ForceTorqueSensorBroadcaster::state_interface_configuration() const
{
    if_config_t	if_config;
    if_config.type  = controller_interface::interface_configuration_type
					  ::INDIVIDUAL;
    if_config.names = _ft_sensor->get_state_interface_names();
    return if_config;
}

ForceTorqueSensorBroadcaster::cb_return_t
ForceTorqueSensorBroadcaster::on_init()
{
  // [A] Wrench stuffs
  // [A-1] Setup frame_id of FT-sensor
    _frame_id = ddynamic_reconfigure2::declare_read_only_parameter(
		    get_node(), "frame_id", "");
    if (_frame_id.empty())
    {
	RCLCPP_ERROR_STREAM(get_node()->get_logger(),
			    "Parameter frame_id is not specified");
	return cb_return_t::ERROR;
    }

  // [A-2] Setup interval period of publishing wrench.
    const auto	pub_rate = ddynamic_reconfigure2::declare_read_only_parameter(
			       get_node(), "publish_rate", 0.0);
    if (pub_rate <= 0.0)
    {
	RCLCPP_ERROR_STREAM(get_node()->get_logger(),
			    "Value of parameter 'publish_rate' is "
			    << pub_rate << ", but must be positive");
	return cb_return_t::ERROR;
    }
    _pub_interval = std::chrono::duration<double>(1.0/pub_rate);

  // [B] Filtering stuffs
  // [B-1] Initialize Butterworth LPF
    _filter.initialize(2, 15.0*_pub_interval.seconds());
    
  // [B-2] Setup dynamic reconfigure server
    _ddr.registerVariable<int>(
	"filter_half_order", int(_filter.half_order()),
	std::bind(&ForceTorqueSensorBroadcaster::set_filter_half_order,
		  this, std::placeholders::_1),
	"Half order of input low pass filter", {1, 5});
    _ddr.registerVariable<double>(
	"filter_cutoff_frequency", _filter.cutoff()/_pub_interval.seconds(),
	std::bind(&ForceTorqueSensorBroadcaster::set_filter_cutoff_frequency,
		  this, std::placeholders::_1),
	"Cutoff frequency of input low pass filter", {0.5, pub_rate});

  // [D] Forward kinematics stuffs
  // [D-1] Load contents of "robot_description" parameter.
    const auto
	robot_desc_name = ddynamic_reconfigure2::declare_read_only_parameter(
			      get_node(),
			      "robot_description", "/robot_description");
    const auto
	robot_desc_string = ddynamic_reconfigure2::declare_read_only_parameter(
			        get_node(), robot_desc_name, "");
    if (robot_desc_string.empty())
    {
	RCLCPP_ERROR_STREAM(get_node()->get_logger(),
			    "Robot description parameter["
			    << robot_desc_name << "] not found");
	return cb_return_t::ERROR;
    }

  // [D-2] Construct KDL tree from robot_description parameter.
    KDL::Tree	tree;
    if (!kdl_parser::treeFromString(robot_desc_string, tree))
    {
	RCLCPP_ERROR_STREAM(get_node()->get_logger(),
			    "Failed to construct kdl tree");
	return cb_return_t::ERROR;
    }

  // [D-3] Get chain from gravity frame to sensor frame.
    const auto
	gravity_frame = ddynamic_reconfigure2::declare_read_only_parameter(
			    get_node(), "gravity_frame", "world");
    if (!tree.getChain(gravity_frame, _frame_id, _chain))
    {
	RCLCPP_ERROR_STREAM(get_node()->get_logger(),
			    "Couldn't create chain from "
			    << gravity_frame + " to " << _frame_id);
	return cb_return_t::ERROR;
    }

  // [D-4] Get names of joints contained in the chain.
    for (u_int i = 0; i < _chain.getNrOfSegments(); ++i)
    {
	const auto&	joint = _chain.getSegment(i).getJoint();
	if (joint.getType() != KDL::Joint::None)
	    _joint_names.push_back(joint.getName());
    }

  // [D-5] Create FK solver for the chain.
    _fksolver.reset(new KDL::ChainFkSolverPos_recursive(_chain));

  // [E] Gravity compensation stuffs
    _ddr.registerVariable<bool>(
	"compensate_gravity", &_compensate_gravity,
	"Compensate gravity if true");
    _mg	= G * ddynamic_reconfigure2::declare_read_only_parameter(
		  get_node(), "effector_mass", 0.0);
    _q	= quaternion_param("rotation");
    _r	= vector_param("mass_center");
    _f0	= vector_param("force_offset");
    _m0	= vector_param("torque_offset");
    
  // [F] Calibration stuffs
    _calib_file = std::string(getenv("HOME"))
		+ "/.ros/aist_ftsensor"
		+ get_node()->get_name() + ".yaml";

    RCLCPP_INFO_STREAM(get_node()->get_logger(),
		       "gravity_frame=" << gravity_frame
		       << ", frame_id=" << _frame_id );
    return cb_return_t::SUCCESS;
}

ForceTorqueSensorBroadcaster::cb_return_t
ForceTorqueSensorBroadcaster::on_configure(const lc_state_t&)
{
  // [A] Wrench stuffs
    const auto
	sensor_name = ddynamic_reconfigure2::declare_read_only_parameter(
			  get_node(), "sensor_name", "");
    _ft_sensor	       = std::make_unique<ft_sensor_t>(sensor_name);
    _wrench_org_pub = std::make_shared<publisher_t<wrench_t> >(
                          *get_node(), "~/wrench_org",
                          rclcpp::SystemDefaultsQoS(),
                          rclcpp::PublisherOptions());
    _wrench_pub	    = std::make_shared<publisher_t<wrench_t> >(
                          *get_node(), "~/wrench",
                          rclcpp::SystemDefaultsQoS(),
                          rclcpp::PublisherOptions());
    
  // [C] JointSate stuffs
    _joint_state_sub = get_node()->create_subscription<joint_state_t>(
			   "joint_states", 1,
			   std::bind(
			       &ForceTorqueSensorBroadcaster::joint_state_cb,
			       this, std::placeholders::_1));

  // [E] Calibration stuffs
    _take_sample = get_node()->create_service<trigger_t>(
		       "~/take_sample",
		       std::bind(&ForceTorqueSensorBroadcaster::take_sample_cb,
				 this,
				 std::placeholders::_1,
				 std::placeholders::_2));
    _compute_calibration = get_node()->create_service<trigger_t>(
			       "~/compute_calibaration",
			       std::bind(&ForceTorqueSensorBroadcaster::
					 compute_calibration_cb, this,
					 std::placeholders::_1,
					 std::placeholders::_2));
    _save_calibration = get_node()->create_service<trigger_t>(
			    "~/save_calibration",
			    std::bind(&ForceTorqueSensorBroadcaster::
				      save_calibration_cb, this,
				      std::placeholders::_1,
				      std::placeholders::_2));
    _clear_samples = get_node()->create_service<trigger_t>(
			 "~/clear_samples",
			 std::bind(&ForceTorqueSensorBroadcaster::
				   clear_samples_cb, this,
				   std::placeholders::_1,
				   std::placeholders::_2));
    _reset_bias = get_node()->create_service<trigger_t>(
		      "~/reset_bias",
		      std::bind(&ForceTorqueSensorBroadcaster::reset_bias_cb,
				this,
				std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return cb_return_t::SUCCESS;
}

ForceTorqueSensorBroadcaster::cb_return_t
ForceTorqueSensorBroadcaster::on_activate(const lc_state_t&)
{
    _ft_sensor->assign_loaned_state_interfaces(state_interfaces_);
    return cb_return_t::SUCCESS;
}

ForceTorqueSensorBroadcaster::cb_return_t
ForceTorqueSensorBroadcaster::on_deactivate(const lc_state_t&)
{
    _ft_sensor->release_interfaces();
    return cb_return_t::SUCCESS;
}

ForceTorqueSensorBroadcaster::ci_return_t
ForceTorqueSensorBroadcaster::update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration&)
{
    if (time < _last_pub_time + _pub_interval)
	return ci_return_t::OK;

  // Get current force-torque values.
    geometry_msgs::msg::WrenchStamped	wrench;
    _ft_sensor->get_values_as_message(wrench.wrench);
    {
	std::lock_guard<std::mutex>	lock(_ft_mtx);

	_ft(0) = wrench.wrench.force.x;
	_ft(1) = wrench.wrench.force.y;
	_ft(2) = wrench.wrench.force.z;
	_ft(3) = wrench.wrench.torque.x;
	_ft(4) = wrench.wrench.torque.x;
	_ft(5) = wrench.wrench.torque.x;
    }

  // Publish unfiltered force-torque signal.
    wrench.header.stamp    = time;
    wrench.header.frame_id = _frame_id;
    _wrench_org_pub->try_publish(wrench);

  // Lookup current joint positions contained in the chain.
    KDL::JntArray	jnt_pos;
    try
    {
	jnt_pos = get_jnt_pos(_joint_names);
    }
    catch (const std::out_of_range& err)
    {
	RCLCPP_WARN_STREAM(get_node()->get_logger(),
			   "joint_state not available yet: " << err.what());
	return ci_return_t::ERROR;
    }

  // Get transform from sensor frame to gravity frame
  // at the current joint positions.
    KDL::Frame	Tgs;
    _fksolver->JntToCart(jnt_pos, Tgs);

  // Get gravity direction w.r.t. sensor frame.
    const vector_t	k = fromKDL(Tgs.M.Inverse(KDL::Vector(0, 0, -1)));

  // Apply low-pass filter to input force-torque signal.
    auto		ft = _filter.filter(_ft);
    const vector_t	f  = ft.head<3>();
    const vector_t	m  = ft.tail<3>();
    
    if (_do_sample)
    {
	take_sample(k, f, m);
	_do_sample = false;
    }
    else if (_do_reset)
    {
	_f0 = f - _q*(_mg*k);
	_m0 = m - _q*(_r.cross(_mg*k));
	_do_reset = false;
    }

    if (_compensate_gravity)
    {
      // Compensate force/torque offsets and gravity.
	ft.head<3>() = _q.inverse()*(f - _f0) - _mg*k;
	ft.tail<3>() = _q.inverse()*(m - _m0) - _r.cross(_mg*k);
    }

  // Publish filtered (and optionally gravity compensated)
  // force-torque signal.
    wrench.wrench.force.x  = ft(0);
    wrench.wrench.force.y  = ft(1);
    wrench.wrench.force.z  = ft(2);
    wrench.wrench.torque.x = ft(3);
    wrench.wrench.torque.y = ft(4);
    wrench.wrench.torque.z = ft(5);
    _wrench_pub->try_publish(wrench);

    return ci_return_t::OK;
}

ForceTorqueSensorBroadcaster::ci_return_t
ForceTorqueSensorBroadcaster::update_reference_from_subscribers(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    return ci_return_t::OK;
}
  /*
std::vector<ForceTorqueSensorBroadcaster::hw_state_if_t>
ForceTorqueSensorBroadcaster::on_export_state_interfaces()
{
    std::vector<hw_state_if_t>	exported_state_interfaces;

    std::vector<std::string>	force_names({params_.interface_names.force.x,
					     params_.interface_names.force.y,
					     params_.interface_names.force.z});
    std::vector<std::string>	torque_names({params_.interface_names.torque.x,
					      params_.interface_names.torque.y,
					      params_.interface_names.torque.z});
    std::string			export_prefix = get_node()->get_name();
    if (!params_.sensor_name.empty())
    {
	const auto	semantic_comp_itf_names = _force_torque_sensor
						->get_state_interface_names();
	std::copy(semantic_comp_itf_names.begin(),
		  semantic_comp_itf_names.begin() + 3, force_names.begin());
	std::copy(semantic_comp_itf_names.begin() + 3,
		  semantic_comp_itf_names.end(), torque_names.begin());

      // Update the prefix and get the proper force and torque names
	export_prefix = export_prefix + "/" + params_.sensor_name;
      // strip "/" and get the second part of the information
      // e.g. /ft_sensor/force.x -> force.x
	std::for_each(force_names.begin(), force_names.end(),
		      [](std::string & name)
		      { name = name.substr(name.find_last_of("/") + 1); });
	std::for_each(torque_names.begin(), torque_names.end(),
		      [](std::string & name)
		      { name = name.substr(name.find_last_of("/") + 1); });
    }
    if (!force_names[0].empty())
    {
	exported_state_interfaces.emplace_back(
	    hardware_interface::StateInterface(
		export_prefix, force_names[0],
		&realtime_publisher_->msg_.wrench.force.x));
    }
    if (!force_names[1].empty())
    {
	exported_state_interfaces.emplace_back(
	    hardware_interface::StateInterface(
		export_prefix, force_names[1],
		&realtime_publisher_->msg_.wrench.force.y));
    }
    if (!force_names[2].empty())
    {
	exported_state_interfaces.emplace_back(
	    hardware_interface::StateInterface(
		export_prefix, force_names[2],
		&realtime_publisher_->msg_.wrench.force.z));
    }
    if (!torque_names[0].empty())
    {
	exported_state_interfaces.emplace_back(
	    hardware_interface::StateInterface(
		export_prefix, torque_names[0],
		&realtime_publisher_->msg_.wrench.torque.x));
    }
    if (!torque_names[1].empty())
    {
	exported_state_interfaces.emplace_back(
	    hardware_interface::StateInterface(
		export_prefix, torque_names[1],
		&realtime_publisher_->msg_.wrench.torque.y));
    }
    if (!torque_names[2].empty())
    {
	exported_state_interfaces.emplace_back(
	    hardware_interface::StateInterface(
		export_prefix, torque_names[2],
		&realtime_publisher_->msg_.wrench.torque.z));
    }
    return exported_state_interfaces;
}
  */
void
ForceTorqueSensorBroadcaster::joint_state_cb(const joint_state_cp& joint_state)
{
    std::lock_guard<std::mutex>	lock(_joint_positions_mtx);

    for (size_t i = 0; i < joint_state->name.size(); ++i)
	_joint_positions[joint_state->name[i]] = joint_state->position[i];
}

void
ForceTorqueSensorBroadcaster::take_sample_cb(const trigger_req&,
					     const trigger_res& res)
{
    take_sample();

    res->success = true;
    res->message = "take_sample succeeded.";
    RCLCPP_INFO_STREAM(get_node()->get_logger(), res->message);
}

void
ForceTorqueSensorBroadcaster::compute_calibration_cb(const trigger_req&,
						     const trigger_res& res)
{
    res->success = compute_calibration();

    if (res->success)
    {
	res->message = "compute_calibration succeeded.";
	RCLCPP_INFO_STREAM(get_node()->get_logger(), res->message);
    }
    else
    {
	res->message = "compute_calibration failed.";
	RCLCPP_ERROR_STREAM(get_node()->get_logger(), res->message);
    }
}

void
ForceTorqueSensorBroadcaster::save_calibration_cb(const trigger_req&,
						  const trigger_res& res)
{
    try
    {
      // Open/create parent directory of the calibration file.
	const auto   dir = _calib_file.substr(0,
					      _calib_file.find_last_of('/'));
	struct stat buf;
	if (stat(dir.c_str(), &buf) && mkdir(dir.c_str(), S_IRWXU))
	    throw std::runtime_error("cannot create " + dir + ": "
						      + strerror(errno));

      // Open calibration file and save calibration results.
	std::ofstream	out(_calib_file.c_str());
	if (!out)
	    throw std::runtime_error("cannot open " + _calib_file + ": "
						    + strerror(errno));
	save_calibration(out);

	res->success = true;
	res->message = "save_calibration succeeded.";
	RCLCPP_INFO_STREAM(get_node()->get_logger(), res->message);
    }
    catch (const std::exception& err)
    {
	res->success = false;
	res->message = std::string("save_calibration failed: ") + err.what();
	RCLCPP_ERROR_STREAM(get_node()->get_logger(), res->message);
    }
}

void
ForceTorqueSensorBroadcaster::clear_samples_cb(const trigger_req&,
					       const trigger_res& res)
{
    clear_samples();

    res->success = true;
    res->message = "clear_samples succeeded.";
    RCLCPP_INFO_STREAM(get_node()->get_logger(), res->message);
}

void
ForceTorqueSensorBroadcaster::reset_bias_cb(const trigger_req&,
					    const trigger_res& res)
{
    reset_bias();

    res->success = true;
    res->message = "reset_bias succeeded.";
    RCLCPP_INFO_STREAM(get_node()->get_logger(), res->message);
}

void
ForceTorqueSensorBroadcaster::take_sample()
{
    _do_sample = true;
}

bool
ForceTorqueSensorBroadcaster::compute_calibration()
{
    using namespace	Eigen;
    using namespace	aist_utility;

    if (_nsamples < 3)
	return false;

  // [0] Compute normal of the plane in which all the torque vectors lie.
    const vector_t	m_mean = _m_sum  / _nsamples;
    const matrix_t	mm_var = _mm_sum / _nsamples - m_mean % m_mean;
    SelfAdjointEigenSolver<matrix_t>	eigensolver(mm_var);
    vector_t		normal = eigensolver.eigenvectors().col(0);

    RCLCPP_INFO_STREAM(get_node()->get_logger(),
		       "RMS error in plane fitting: "
		       << std::sqrt(eigensolver.eigenvalues()(0)));

  // [1] Compute similarity transformation from gravity to observed torque.
  //   Note: Since rank(km_var) = 2, its third singular value is zero.
    const vector_t	k_mean = _k_sum  / _nsamples;
    const matrix_t	km_var = skew(normal) * (_km_sum / _nsamples -
						 k_mean % m_mean);
    JacobiSVD<matrix_t>	svd(km_var, ComputeFullU | ComputeFullV);

    matrix_t	Ut = svd.matrixU().transpose();
    if (Ut.determinant() < 0)
	Ut.row(2) *= -1;
    matrix_t	V = svd.matrixV();
    if (V.determinant() < 0)
	V.col(2) *= -1;
    _q = V * Ut;					// rotation

    const auto	k_var = _k_sqsum / double(_nsamples) - k_mean.squaredNorm();
    const auto	scale = (svd.singularValues()(0) +
			 svd.singularValues()(1)) / k_var;
    _m0 = m_mean - scale * (_q * normal.cross(k_mean));	// torque offset

  // [2] Compute transformation from gravity to observed force.
    const vector_t	f_mean = _f_sum / _nsamples;
    const matrix_t 	kf_var = (_kf_sum / _nsamples - k_mean % f_mean);

  // If the effector mass value becomes negative, reverse the normal direction
  // and fix the rotation.
    if ((_q * kf_var).trace() < 0)
    {
	normal	  *= -1;	// Reverse the normal direction.
	Ut.row(0) *= -1;	// Reverse first two rows of Ut so that
	Ut.row(1) *= -1;	// the sign of SVD to be reversed while
				// keeping those of singular vlues.
	_q  = V * Ut;		// Recompute rotation.
    }
    _mg = (_q * kf_var).trace() / k_var;		// effector mass
    _r  = (scale / _mg) * normal;			// mass center
    _f0 = f_mean - _mg * (_q * k_mean);			// force offset

  // Evaluate residual error.
    // const auto	f_var = f_sqsum/_nsamples - f_avg.squaredNorm();
    // RCLCPP_INFO_STREAM("(aist_ftsensor_controller) force residual error = "
    // 		    << std::sqrt(f_var/k_var - _mg*_mg)
    // 		    << "(Newton)");

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "calibration computed");

    return true;
}

void
ForceTorqueSensorBroadcaster::save_calibration(std::ostream& out) const
{
    const auto	name = get_node()->get_name();

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << name << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "frame_id" << YAML::Value << _frame_id;
    emitter << YAML::Key << "effector_mass" << YAML::Value << _mg/G;
    emitter << YAML::Key << "rotation"	<< YAML::Value << YAML::Flow
	    << YAML::BeginSeq
	    << _q.x() << _q.y() << _q.z() << _q.w()
	    << YAML::EndSeq;
    emitter << YAML::Key << "force_offset" << YAML::Value << YAML::Flow
	    << YAML::BeginSeq
	    << _f0(0) << _f0(1) << _f0(2)
	    << YAML::EndSeq;
    emitter << YAML::Key << "torque_offset" << YAML::Value << YAML::Flow
	    << YAML::BeginSeq
	    << _m0(0) << _m0(1) << _m0(2)
	    << YAML::EndSeq;
    emitter << YAML::Key << "mass_center" << YAML::Value << YAML::Flow
	    << YAML::BeginSeq
	    << _r(0) << _r(1) << _r(2)
	    << YAML::EndSeq;
    emitter << YAML::Key << "filter_half_order"
	    << YAML::Value << _filter.half_order();
    emitter << YAML::Key << "filter_cutoff_frequency"
	    << YAML::Value << _filter.cutoff()/_pub_interval.seconds();
    emitter << YAML::EndMap;
    emitter << YAML::EndMap;

    out << emitter.c_str() << std::endl;

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "calibration saved");
}

void
ForceTorqueSensorBroadcaster::clear_samples()
{
    _nsamples = 0;
    _k_sum    = vector_t::Zero();
    _f_sum    = vector_t::Zero();
    _m_sum    = vector_t::Zero();
    _k_sqsum  = 0;
    _kf_sum   = matrix_t::Zero();
    _km_sum   = matrix_t::Zero();
    _mm_sum   = matrix_t::Zero();

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "samples cleared");
}

void
ForceTorqueSensorBroadcaster::reset_bias()
{
    _do_reset = true;
}

void
ForceTorqueSensorBroadcaster::take_sample(const vector_t& k,
					  const vector_t& f,
					  const vector_t& m)
{
    using	namespace aist_utility;

    ++_nsamples;
    _k_sum   += k;
    _f_sum   += f;
    _m_sum   += m;
    _k_sqsum += k.squaredNorm();
    _kf_sum  += k % f;
    _km_sum  += k % m;
    _mm_sum  += m % m;

    // _fout << k.transpose() << std::endl;
    // _fout << f.transpose() << std::endl;
    // _fout << m.transpose() << std::endl << std::endl;
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
		       _nsamples << "-th sample taken");
}

void
ForceTorqueSensorBroadcaster::set_filter_half_order(int half_order)
{
    std::lock_guard<std::mutex> lock(_ft_mtx);

    _filter.initialize(half_order, _filter.cutoff());
    _filter.reset(_ft);
}

void
ForceTorqueSensorBroadcaster::set_filter_cutoff_frequency(
    double cutoff_frequency)
{
    std::lock_guard<std::mutex> lock(_ft_mtx);

    _filter.initialize(_filter.half_order(),
		       cutoff_frequency*_pub_interval.seconds());
    _filter.reset(_ft);
}

ForceTorqueSensorBroadcaster::vector_t
ForceTorqueSensorBroadcaster::vector_param(const std::string& name)
{
    const auto	v = ddynamic_reconfigure2::declare_read_only_parameter(
			get_node(), name,
			std::vector<double>({0.0, 0.0, 0.0}));
    if (v.size() == 3)
	return {v[0], v[1], v[2]};

    return {0.0, 0.0, 0.0};
}

ForceTorqueSensorBroadcaster::quaternion_t
ForceTorqueSensorBroadcaster::quaternion_param(const std::string& name)
{
    const auto	v = ddynamic_reconfigure2::declare_read_only_parameter(
			get_node(), name,
			std::vector<double>({0.0, 0.0, 0.0, 1.0}));
    if (v.size() == 4)
	return {v[3], v[0], v[1], v[2]};

    return {1.0, 0.0, 0.0, 0.0};
}

KDL::JntArray
ForceTorqueSensorBroadcaster::get_jnt_pos(
    const std::vector<std::string>& jnt_name) const
{
    KDL::JntArray	jnt_pos(u_int(jnt_name.size()));

    std::lock_guard<std::mutex>	lock(_joint_positions_mtx);

    for (u_int i = 0; i < jnt_name.size(); ++i)
	jnt_pos(i) = _joint_positions.at(jnt_name[i]);

    return jnt_pos;
}
}	// namespace aist_ftsensor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(aist_ftsensor::ForceTorqueSensorBroadcaster,
		       controller_interface::ChainableControllerInterface)
