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
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/srv/trigger.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <cstdlib>		// for std::getenv()
#include <sys/stat.h>		// for mkdir()
#include <aist_utility/eigen.h>
#include <aist_utility/butterworth_lpf.h>

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
    for (size_t i = 0; i < joints.rows(); ++i)
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
    using cb_return_t	= controller_interface::CallbackReturn;
    using ci_return_t	= controller_interface::return_type;
    using lc_state_t	= rclcpp_lifecylce::State;
    
    using interface_t	= hardware_interface::ForceTorqueSensorInterface;
    using wrench_t	= geometry_msgs::msg::WrenchStamped;
    using joint_state_t = sensor_msgs::msg::JointState;
    using joint_state_cp= joint_state_t::ConstSharedPtr;

    using handle_t		= hardware_interface::ForceTorqueSensorHandle;
    using fksolver_p	= std::unique_ptr<KDL::ChainFkSolverPos>;
    using controller_t	= ForceTorqueSensorBroadcaster;
    using vector_t	= Eigen::Vector3d;
    using matrix_t	= Eigen::Matrix3d;
    using quaternion_t	= Eigen::Quaterniond;
    using ft_t		= Eigen::Matrix<double, 6, 1>;
    using filter_t		= aist_utility::ButterworthLPF<double, ft_t>;
    using ddr_t		= ddynamic_reconfigure2::DDynamicReconfigure;
    
    using ft_sensor_t	= semantic_components::ForceTorqueSensor;

    using trigger_t	= std_msgs::srv::Trigger;
    using trigger_req	= trigger_t::Request::SharedPtr;
    using trigger_res	= trigger_t::Response::SharedPtr;
    
    template <class MSG>
    using publisher_p	= typename rclcpp::Publisher<MSG>;
    template <class MSG>
    using rt_publisher_t= typename realtime_tools::RealtimePublisher<MSG>;
    template <class MSG>
    using rt_publisher_p= std::unique_ptr<rt_publisher_t<MSG> >;
    template <class MSG>
    using subscription_p= typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using service_p	= typename rclcpp::Service<SRV>::SharedPtr;
    
    constexpr static double	G = 9.80665;

  public:
		ForceTorqueSensorBroadcaster()				;

    cb_return_t	on_init(const lc_state& prev_state)			;
    cb_return_t	on_configure(const lc_state& prev_state)		;
    cb_return_t	on_activate(const lc_state& prev_state)			;
    cb_return_t	on_deactivate(const lc_state& prev_state)		;

    ci_return_t	update_and_write_commands(
		    const rclcpp::Time& time,
		    const rclcpp::Duration& period)			;
    ci_return_t	update_reference_from_subscribers(
		    const rclcpp::Time& time,
		    const rclcpp::Duration& period)			;

    void	joint_state_cb(const joint_state_cp& joint_state)	;
    bool	take_sample_cb(const trigger_req& req,
			       const trigger_res& res)			;
    bool	compute_calibration_cb(const trigger_req& req,
				       const trigger_res& res);
    bool	save_calibration_cb(const trigger_req& req,
				    const trigger_res& res)		;
    bool	clear_samples_cb(const trigger_req& req,
				 const trigger_res& res)		;
    bool	reset_bias_cb(const trigger_req& req,
			      const trigger_res& res)			;

    const KDL::Tree&	get_tree()				const	;
    void		get_jnt_pos(const std::vector<std::string>& jnt_name,
				    KDL::JntArray& jnt_pos)	const	;

  private:
    void	take_sample()						;
    bool	compute_calibration()					;
    void	save_calibration(std::ostream& out)		const	;
    void	clear_samples()						;
    void	reset_bias()						;

    void	take_sample(const vector_t& k,
			    const vector_t& f, const vector_t& m)	;
    void	set_filter_half_order(int half_order)			;
    void	set_filter_cutoff_frequency(double cutoff_frequency)	;

    vector_t	vector_param(const std::string& name)		const	;
    quaternion_t
		quaternion_param(const std::string& name)	const	;

  private:
    std::unique_ptr<ft_sensor_t>	_ft_sensor;
    const std::string			_frame_id;

  // JointState stuffs
    KDL::Tree				_tree;
    const subscription_p<joint_state_t>	_joint_state_sub;
    std::map<std::string, double>	_joint_positions;
    mutable std::mutex			_joint_state_mtx;

  // Wrench stuffs
    publisher_p<wrench_t>		_wrench_org_pub;
    publisher_p<wrench_t>		_wrench_pub;
    rt_publisher_p<wrench_t>		_wrench_rt_pub;
    const rclcpp::Duration		_pub_interval;
    rclcpp::Time			_last_pub_time;

  // Calibration stuffs
    const service_p<trigger_t>		_take_sample;
    const service_p<trigger_t>		_compute_calibration;
    const service_p<trigger_t>		_save_calibration;
    const service_p<trigger_t>		_clear_samples;
    const service_p<trigger_t>		_reset_bias;
    std::string				_calib_file;

    ddr_t				_ddr;

  // Filtering stuffs
    ft_t				_ft;
    filter_t				_filter;
    mutable std::mutex			_ft_mtx;

  // Forward kinematics stuffs
    const controller_t&			_controller;
    KDL::Chain				_chain;
    std::vector<std::string>		_joint_names;
    KDL::JntArray			_joint_positions;
    fksolver_p				_fksolver;

  // Variables retrieved from parameter server
    bool				_compensate_gravity;
    double				_mg;		// effector mass
    quaternion_t			_q;		// rotation
    vector_t				_r;		// mass center
    vector_t				_f0;		// force offset
    vector_t				_m0;		// torque offset

  // Calibration stuffs
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
};

ForceTorqueSensorBroadcaster::ForceTorqueSensorBroadcaster()
    :controller_interface::ChainableControllerInterface(),
     _ft_sensor(),
     _frame_id(),
     
     _tree(),
     _joint_state_sub(),
     _joint_positions(),
     _joint_state_mtx(),

     _wrench_org_pub(),
     _wrench_pub(),
     _wrench_rt_pub(),
     _pub_interval(),
     _last_pub_time(),
     
     _take_sample(),
     _compute_calibration(),
     _save_calibration(),
     _clear_samples(),
     _reset_bias(),
     _calib_file(),
     
     _ddr(_nh),

     _ft(ft_t::Zero()),
     _filter(2, 15.0*_pub_interval.toSec()),
     _ft_mtx(),
     
     _controller(controller),
     _chain(),
     _joint_names(),
     _joint_positions(),
     _fksolver(),
     
     _compensate_gravity(false),
     _mg(G*_nh.param<double>("effector_mass", 0.0)),
     _q(quaternion_param("rotation")),
     _r(vector_param("mass_center")),
     _f0(vector_param("force_offset")),
     _m0(vector_param("torque_offset")),
     
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
     _fout()
{
}

ForceTorqueSensorBroadcaster::cb_result_t
ForceTorqueSensorBroadcaster::on_init(const lc_state& prev_state)
{
  // Load contents of "robot_description" parameter.
    const auto
	param_name = ddynamic_reconfigure2::
			declare_read_only_parameter<std::string>(
			    "robot_description", "/robot_description");
    const auto
	robot_desc_string = ddynamic_reconfigure2::
				declare_read_only_parameter<std::string>(
				    paramname, "");
    if (robot_desc_string == "")
    {
	RCLCPP_ERROR_STREAM(get_loggeer(), "Robot description parameter["
			    << param_name << "] not found");
	return cb_return_t::ERROR;
    }

  // Construct KDL tree from robot_description parameter.
    if (!kdl_parser::treeFromString(robot_desc_string, _tree))
    {
	RCLCPP_ERROR_STREAM(get_logger(), "Failed to construct kdl tree");
	return cb_return_t::ERROR;
    }

  // Get calibration file name from parameter server.
    _calib_file = std::string(getenv("HOME"))
		+ "/.ros/aist_ftsensor"
		+ get_node()->get_name() + ".yaml";

    return cb_return_t::SUCCESS;
}

ForceTorqueSensorBroadcaster::cb_result_t
ForceTorqueSensorBroadcaster::on_configure(const lc_state& prev_state)
{
  // Get publishing period.
    const auto	pub_rate = controller_nh.param<double>("publish_rate", 0.0);
    if (pub_rate <= 0.0)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "Value of parameter 'publish_rate' is "
			    << pub_rate << ", but must be positive.");
	return false;
    }

    _ft_sensor = std::make_unique<ft_sensor_t>(ft_sensor_t());

    _take_sample = get_node()->create_service<trigger_t>(
		       "~/take_sample",
		       std::bind(&ForceToruqeSensorController::take_sample_cb,
				 this,
				 std::placeholders::_1,
				 std::placeholders::_2));
    _compute_calibration = get_node()->create_service<trigger_t>(
			       "~/compute_calibaration",
			       std::bind(&ForceToruqeSensorController::
					 compute_calibration_cb, this,
					 std::placeholders::_1,
					 std::placeholders::_2));
    _save_calibration = get_node()->create_service<trigger_t>(
			    "~/save_calibration",
			    std::bind(&ForceToruqeSensorController::
				      save_calibration_cb, this,
				      std::placeholders::_1,
				      std::placeholders::_2));
    _clear_samples = get_node()->create_service<trigger_t>(
			 "~/clear_samples",
			 std::bind(&ForceToruqeSensorController::
				   clear_samples_cb, this,
				   std::placeholders::_1,
				   std::placeholders::_2));
    _reset_bias = get_node()->create_service<trigger_t>(
		      "~/reset_bias",
		      std::bind(&ForceToruqeSensorController::reset_bias,
				this,
				std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return cb_return_t::SUCCESS;
}

ForceTorqueSensorBroadcaster::cb_result_t
ForceTorqueSensorBroadcaster::on_activate(const lc_state& prev_state)
{
    _ft_sensor->assign_loaned_state_interfaces(_state_interfaces);
    return cb_return_t::SUCCESS;
}

ForceTorqueSensorBroadcaster::cb_result_t
ForceTorqueSensorBroadcaster::on_deactivate(const lc_state& prev_state)
{
    _ft_sensor->release_interfaces();
    return cb_return_t::SUCCESS;
}

ForceTorqueSensroBoradCaster::ci_return_t
ForceTorqueSensorBroadcaster::update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
    if (time < _last_pub_time + _pub_interval)
	return;

    wrench_t	wrench;
    _ft_sensor->get_values_as_message(wrench);
    
  // Publish unfiltered force-torque signal.
    if (_wrench_org_pub->trylock())
    {
	_wrench_org_pub->msg_.wrench = wrench;
	_wrench_org_pub->msg_.header.stamp    = time;
	_wrench_org_pub->msg_.header.frame_id = _frame_id;
	_wrench_org_pub->unlockAndPublish();
    }

  // Lookup current joint positions contained in the chain.
    try
    {
	_controller.get_jnt_pos(_joint_names, _joint_positions);
    }
    catch (const std::out_of_range& err)
    {
	RCLCPP_WARN_STREAM('(' << _nh.getNamespace()
			<< ") joint_state not available yet: " << err.what());
	return;
    }

  // Get transform from sensor frame to gravity frame
  // for current joint positions.
    KDL::Frame	Tgs;
    _fksolver->JntToCart(_joint_positions, Tgs);

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
    if (_pub->trylock())
    {
	_pub->msg_.header.stamp    = time;
	_pub->msg_.header.frame_id = _hw_handle.getFrameId();
	_pub->msg_.wrench.force.x  = ft(0);
	_pub->msg_.wrench.force.y  = ft(1);
	_pub->msg_.wrench.force.z  = ft(2);
	_pub->msg_.wrench.torque.x = ft(3);
	_pub->msg_.wrench.torque.y = ft(4);
	_pub->msg_.wrench.torque.z = ft(5);

	_pub->unlockAndPublish();
	_last_pub_time = time;
    }

    return ci_return_t::OK;
}

ForceTorqueSensroBoradCaster::ci_return_t
ForceTorqueSensorBroadcaster::update_reference_from_subscribers(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
    return ci_return_t::OK;
}

void
ForceTorqueSensorBroadcaster::joint_state_cb(const joint_state_cp& joint_state)
{
    std::lock_guard<std::mutex>	lock(_joint_state_mtx);

    for (size_t i = 0; i < joint_state->name.size(); ++i)
	_joint_positions[joint_state->name[i]] = joint_state->position[i];
}

void
ForceTorqueSensorBroadcaster::take_sample_cb(const trigger_req& req,
					     const trigger_res& res)
{
    for (const auto& sensor : _sensors)
	sensor->take_sample();

    res->success = true;
    res->message = "take_sample succeeded.";
    RCLCPP_INFO_STREAM(res.message);
}

void
ForceTorqueSensorBroadcaster::compute_calibration_cb(const trigger_req& req,
						     const trigger_res& res)
{
    for (const auto& sensor : _sensors)
	if (!sensor->compute_calibration())
	{
	    res.success = false;
	    res.message = "compute_calibration failed.";
	    RCLCPP_ERROR_STREAM("(aist_ftsensor_controller) " << res.message);

	    return true;
	}

    res->success = true;
    res->message = "compute_calibration succeeded.";
    RCLCPP_INFO_STREAM(res.message);
}

void
ForceTorqueSensorBroadcaster::save_calibration_cb(const trigger_req& req,
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
	for (const auto& sensor : _sensors)
	    sensor->save_calibration(out);

	res->success = true;
	res->message = "save_calibration succeeded.";
	RCLCPP_INFO_STREAM("(aist_ftsensor_controller) " << res.message);
    }
    catch (const std::exception& err)
    {
	res->success = false;
	res->message = std::string("save_calibration failed: ") + err.what();
	RCLCPP_ERROR_STREAM(res.message);
    }
}

void
ForceTorqueSensorBroadcaster::clear_samples_cb(const trigger_req& req,
					       const trigger_res& res)
{
    for (const auto& sensor : _sensors)
	sensor->clear_samples();

    res->success = true;
    res->message = "clear_samples succeeded.";
    RCLCPP_INFO_STREAM(res.message);
}

void
ForceTorqueSensorBroadcaster::reset_bias_cb(const trigger_req& req,
					    const trigger_res& res)
{
    for (const auto& sensor : _sensors)
	sensor->reset_bias();

    res->success = true;
    res->message = "reset_bias succeeded.";
    RCLCPP_INFO_STREAM(res.message);
}

const KDL::Tree&
ForceTorqueSensorBroadcaster::get_tree() const
{
    return _tree;
}

void
ForceTorqueSensorBroadcaster::get_jnt_pos(
    const std::vector<std::string>& jnt_name, KDL::JntArray& jnt_pos) const
{
    std::lock_guard<std::mutex>	lock(_joint_state_mtx);

    for (size_t i = 0; i < jnt_name.size(); ++i)
	jnt_pos(i) = _joint_positions.at(jnt_name[i]);
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

    RCLCPP_INFO_STREAM("(aist_ftsensor_controller) RMS error in plane fitting: "
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

    const auto	k_var = _k_sqsum / _nsamples - k_mean.squaredNorm();
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

    RCLCPP_INFO_STREAM('(' << _nh.getNamespace() << ") calibration computed");

    return true;
}

void
ForceTorqueSensorBroadcaster::save_calibration(std::ostream& out) const
{
    const auto	ns   = _nh.getNamespace();
    const auto	name = ns.substr(ns.find_last_of('/') + 1);

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
	    << YAML::Value << _filter.cutoff()/_pub_interval.toSec();
    emitter << YAML::EndMap;
    emitter << YAML::EndMap;

    out << emitter.c_str() << std::endl;

    RCLCPP_INFO_STREAM('(' << _nh.getNamespace() << ") calibration saved");
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

    RCLCPP_INFO_STREAM('(' << _nh.getNamespace() << ") samples cleared");
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
    RCLCPP_INFO_STREAM('(' << _nh.getNamespace() << ") "
		    << _nsamples << "-th sample taken");
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
		       cutoff_frequency*_pub_interval.toSec());
    _filter.reset(_ft);
}

ForceTorqueSensorBroadcaster::vector_t
ForceTorqueSensorBroadcaster::vector_param(const std::string& name) const
{
    if (_nh.hasParam(name))
    {
	std::vector<double>	v;
	_nh.getParam(name, v);

	if (v.size() == 3)
	{
	    vector_t	vec;
	    vec << v[0], v[1], v[2];
	    return vec;
	}
    }

    return vector_t::Zero();
}

ForceTorqueSensorBroadcaster::quaternion_t
ForceTorqueSensorBroadcaster::quaternion_param(const std::string& name) const
{
    if (_nh.hasParam(name))
    {
	std::vector<double>	v;
	_nh.getParam(name, v);

	if (v.size() == 4)
	    return {v[3], v[0], v[1], v[2]};
    }

    return {1.0, 0.0, 0.0, 0.0};
}

ForceTorqueSensorBroadcaster::Sensor
			   ::Sensor(interface_t* hw,
				    ros::NodeHandle& root_nh,
				    const ros::NodeHandle& controller_nh,
				    const std::string& name,
				    double pub_rate,
				    const controller_t& controller)
    :_hw_handle(hw->getHandle(name)),
     _nh(controller_nh, "sensors/" + name),
     _frame_id(
	 _nh.param<std::string>(
	     "frame_id",
	     _hw_handle.getFrameId().substr(
		 0, _hw_handle.getFrameId().find("_controller")))),
     _pub_org(new publisher_t(root_nh, name + "_org", 4)),
     _pub(new publisher_t(root_nh, name, 4)),
     _pub_interval(1.0/pub_rate),
     _last_pub_time(0),
{
    if (_frame_id == "")
	throw std::runtime_error("Parameter frame_id is not specified");

  // Get chain from gravity frame to sensor frame.
    const auto	gravity_frame = controller_nh.param<std::string>(
				    "gravity_frame", "world");
    if (!_controller.get_tree().getChain(gravity_frame, _frame_id, _chain))
	throw std::runtime_error("Couldn't create chain from "
				 + gravity_frame + " to " + _frame_id);

  // Get names of joints contained in the chain.
    for (size_t i = 0; i < _chain.getNrOfSegments(); ++i)
    {
	const auto&	joint = _chain.getSegment(i).getJoint();
	if (joint.getType() != KDL::Joint::None)
	    _joint_names.push_back(joint.getName());
    }
    _joint_positions.resize(_joint_names.size());

  // Create FK solver for the chain.
    _fksolver.reset(new KDL::ChainFkSolverPos_recursive(_chain));

  // Setup dynamic reconfigure server
    _ddr.registerVariable<int>(
	"filter_half_order", _filter.half_order(),
	boost::bind(&Sensor::set_filter_half_order, this, _1),
	"Half order of input low pass filter", 1, 5);
    _ddr.registerVariable<double>(
	"filter_cutoff_frequency", _filter.cutoff()/_pub_interval.toSec(),
	boost::bind(&Sensor::set_filter_cutoff_frequency, this, _1),
	"Cutoff frequency of input low pass filter", 0.5, pub_rate);
    _ddr.registerVariable<bool>(
	"compensate_gravity", &_compensate_gravity,
	"Compensate gravity if true", false, true);
    _ddr.publishServicesTopicsAndUpdateConfigData();

    RCLCPP_INFO_STREAM('(' << _nh.getNamespace()
		    << ") got sensor. gravity_frame=" << gravity_frame
		    << ", frame_id=" << _frame_id );
}

void
ForceTorqueSensorBroadcaster::Sensor::starting(const ros::Time& time)
{
    _last_pub_time = time;
}


}	// namespace aist_ftsensor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(aist_ftsensor::ForceTorqueSensorBroadcaster,
		       controller_interface::ChainableControllerInterface)
