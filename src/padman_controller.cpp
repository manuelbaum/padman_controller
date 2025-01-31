// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "padman_controller/padman_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <filesystem>

#include "controller_interface/helpers.hpp"
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
// #include <pinocchio/algorithm/joint-configuration.hpp>
// #include <pinocchio/algorithm/kinematics.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = padman_controller::PadmanController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace padman_controller
{
PadmanController::PadmanController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PadmanController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<padman_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PadmanController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&PadmanController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.joints[0];
  state_publisher_->unlock();

  // read and set gain parameters
  //params_.joint1.p



  //get_node()->declare_parameter("kp", std::vector<double>{10.0, 10.0, 2.5});

  // Setup kinematics with pinocchio
  const auto package_share_path = ament_index_cpp::get_package_share_directory("padman_hw");
  const auto urdf_path = std::filesystem::path(package_share_path) /"urdf"/ "padman.urdf";
  //const auto srdf_path = std::filesystem::path(package_share_path) / "ur_robot_model" / "ur5_gripper.srdf";

  // Create a set of Pinocchio models and data.
  pinocchio::urdf::buildModel(urdf_path, pinocchio_model);

  pinocchio::GeometryModel visual_model;
  pinocchio::urdf::buildGeom(pinocchio_model, urdf_path, pinocchio::VISUAL, visual_model);

  
  pinocchio::urdf::buildGeom(pinocchio_model, urdf_path, pinocchio::COLLISION, pinocchio_collision_model);
  pinocchio_collision_model.addAllCollisionPairs();
  //pinocchio::srdf::removeCollisionPairs(model, collision_model, srdf_path);

  pinocchio_data= pinocchio::Data(pinocchio_model);



  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void PadmanController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->joint_names.size() == params_.joints.size())
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu , but expected %zu joints in command. Ignoring message.",
      msg->joint_names.size(), params_.joints.size());
  }
}

controller_interface::InterfaceConfiguration PadmanController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PadmanController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/position");
    
  }
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/velocity");
    
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn PadmanController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for exemplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

  time_activate = get_node()->now();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PadmanController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    bool is_set_successfully = command_interfaces_[i].set_value(0);
    if (!is_set_successfully){
        RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set_value in controller");
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PadmanController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{

  if (param_listener_->is_old(params_)) {
  params_ = param_listener_->get_params();
}

  const std::vector< double > kp_std = params_.kp;//.as_double_array();
  Eigen::Vector3d kp(kp_std.data());

  const std::vector< double > kd_std = params_.kd;//.as_double_array();
  Eigen::Vector3d kd(kd_std.data());
  // kp = Eigen::VectorXd(3);
  // //kp <<params_.gains.joint1.p, params_.gains.joint2.p, params_.gains.joint3.p;
  // kp <<20.0, 20.0, 5.0;

  // kd = Eigen::VectorXd(3);
  // //kd <<params_.gains.joint1.d, params_.gains.joint2.d, params_.gains.joint3.d;
  // kd <<0.1, 0.1, 0.1;

  Eigen::VectorXd q(pinocchio_model.nv);
  Eigen::VectorXd dq(pinocchio_model.nv);
  int pos_ind=0;
  int vel_ind=1;
  int num_joints_=3;
  for (std::size_t i = 0; i < 3; i++)
    {
      q(i) = state_interfaces_[pos_ind * num_joints_ + i].get_value();
      dq(i) = state_interfaces_[vel_ind * num_joints_ + i].get_value();
      //q(i)=get_state((std::string("joint")+std::to_string(i+1)+std::string("/position")));
    }
  // compute robot jacobian
  
  //q << 0.0, 0.0, 0.0;
  //std::cout << "Joint configuration: " << std::endl << "q:"<<std::endl<<q << std::endl << "dq:"<<std::endl << dq << std::endl;

  // Get the frame ID of the end effector for later lookups.
  const auto ee_frame_id = pinocchio_model.getFrameId("ee1");
  //const auto base_link_frame_id = pinocchio_model.getFrameId("base_link");

  // Perform forward kinematics and get a transform.
  pinocchio::framesForwardKinematics(pinocchio_model, pinocchio_data, q);
  //std::cout << "Frame transform: " << std::endl << pinocchio_data.oMf[ee_frame_id] << std::endl;

  // Get a Jacobian at a specific frame.
  Eigen::MatrixXd J(6, pinocchio_model.nv);
  J.setZero();
  pinocchio::computeFrameJacobian(pinocchio_model, pinocchio_data, q, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J); //pinocchio::LOCAL_WORLD_ALIGNED


    std::stringstream ssj;
  ssj << "J: " << std::endl << J << std::endl << std::endl;
  RCLCPP_INFO_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          ssj.str().c_str());

ssj << "kp: " << std::endl << kp << std::endl << std::endl;
    RCLCPP_INFO_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          ssj.str().c_str());
  //J = J.topRows(3).eval();
  // std::cout << "Frame Jacobian: " << std::endl << ee_jacobian << std::endl << std::endl;

    // std::cout << "Joint names in the model:" << std::endl;
    // for (pinocchio::Model::JointIndex i = 1; i < pinocchio_model.joints.size(); ++i) {
    //     std::cout << i << ": " << pinocchio_model.names[i] << std::endl;
    // }




  // Eigen::VectorXd kp(3);
  // kp <<10.0, 10.0, 4.0;

  // Eigen::VectorXd kd(3);
  // kd <<0.01, 0.01, 0.0001;




  // Perform the forward kinematics over the kinematic tree
  pinocchio::forwardKinematics(pinocchio_model,pinocchio_data,q);
      // Update the frames' positions
    pinocchio::updateFramePlacements(pinocchio_model, pinocchio_data);
  // Print out the placement of each joint of the kinematic tree
  // std::cout<<"Forward kinamtics for joints: "<<pinocchio_model.njoints<<std::endl;
  Eigen::Vector3d x = pinocchio_data.oMf[ee_frame_id].translation();
  // std::cout << std::fixed << std::setprecision(2)
  //             << pinocchio_data.oMf[ee_frame_id].translation().transpose()
  //             << std::endl;

  Eigen::Vector3d x_d(3);
  x_d<<0.08, 0.14, -0.07;

  double t = (time - time_activate).seconds();
  double seconds_per_target = 4.0;
  int n_targets = 2;
  int i_target = (int) std::floor(std::fmod(t, n_targets*seconds_per_target) / seconds_per_target);


  double ratio_elapsed = std::fmod(t, seconds_per_target) / seconds_per_target;
  double w_0, w_1;
  if(i_target == 0){
    w_0 = 1.0-ratio_elapsed;
    w_1 =     ratio_elapsed;
  } else {
    w_0 =     ratio_elapsed;
    w_1 = 1.0-ratio_elapsed;
  }

  Eigen::Matrix3d X;
  X << 0, 0, 0.2,
       0, 0, 0.00,
       0, 0.05, 0;

  //x_d = x_d+X.row(i_target).transpose();
  x_d = x_d+(X.row(0)*w_0 + X.row(1)*w_1).transpose();

  Eigen::VectorXd ddx_d(6);
  ddx_d << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ddx_d.head<3>() = x_d - x;

  // Eigen::Vector3d ddx_d(3);
  // ddx_d = 10.0*(x_d - x);

  // //Eigen::VectorXd tau = ee_jacobian.transpose()*f;
  // Eigen::Matrix3d J_inv = J.topRows(3).inverse();
  
  ssj << "J: " << std::endl << J << std::endl << std::endl;
  RCLCPP_INFO_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          ssj.str().c_str());
  // Eigen::Vector3d ddq_d = J_inv*ddx_d;

  Eigen::Matrix3d M = pinocchio::crba(pinocchio_model, pinocchio_data, q);
  M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();

      ssj << "M: " << std::endl << M << std::endl << std::endl;
      RCLCPP_INFO_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 1000,
              ssj.str().c_str());

      ssj << "M_inv: " << std::endl << M.inverse() << std::endl << std::endl;
      RCLCPP_INFO_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 1000,
              ssj.str().c_str());

      ssj << "M_inv sandwich: " << std::endl << J*M.inverse()*J.transpose() << std::endl << std::endl;
      RCLCPP_INFO_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 1000,
              ssj.str().c_str());

  Eigen::MatrixXd M_x = (J*M.inverse()*J.transpose()).inverse(); //operational space mass matrix

      ssj << "M_x: " << std::endl << M_x << std::endl << std::endl;
      RCLCPP_INFO_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 1000,
              ssj.str().c_str());

  Eigen::VectorXd f_d = ddx_d;//M_x*ddx_d;

      ssj << "f_d: " << std::endl << f_d << std::endl << std::endl;
      RCLCPP_INFO_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 1000,
              ssj.str().c_str());

  Eigen::Vector3d tau_task = (J.transpose()*f_d);//M*ddq_d;

        ssj << "tau_task: " << std::endl << tau_task << std::endl << std::endl;
      RCLCPP_INFO_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 1000,
              ssj.str().c_str());

  Eigen::Vector3d tau = kp.array() * tau_task.array() - kd.array() * dq.array();
  //tau << 0.0, 0.0, 0.0;

  //Eigen::VectorXd tau = 0.1*-q;
//  use jacobian to compute force into a specific direction, e.g. up
//  then rotate the force with sin/cos
  //std::cout<<"Pre normalization: "<<tau(0)<<" "<<tau(1)<<" "<<tau(2)<<" "<<"\n"<<std::endl;
  //std::cout<<"kp "<<kp(0)<<" "<<kp(1)<<" "<<kp(2)<<" "<<"\n"<<std::endl;
  //std::cout<<"-dq "<<-dq(0)<<" "<<-dq(1)<<" "<<-dq(2)<<" "<<"\n"<<std::endl;
  //std::cout<<"kd "<<kd(0)<<" "<<kd(1)<<" "<<kd(2)<<" "<<"\n"<<std::endl;
  //tau = tau.array() * kp.array();
  //tau = tau.array() + (kd.array() * -dq.array()); 
  //std::cout<<"New desired torque: "<<tau(0)<<" "<<tau(1)<<" "<<tau(2)<<" "<<"\n"<<std::endl;
  // auto current_ref = input_ref_.readFromRT();
  // std::cout<<"tau "<<tau(0)<<" "<<tau(1)<<" "<<tau(2)<<" "<<"\n"<<std::endl;
  // double t = time.seconds();
  // Eigen::VectorXd tau(3);
  // tau << t, t, t;
  // tau = tau / 4.0;
  // tau = tau.array().sin();
  // tau = 0.05 * tau;

  
  //tau = M*tau;

  std::stringstream ssm;
  ssm << "Mass matrix: " << std::endl << M << std::endl << std::endl;
  RCLCPP_INFO_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          ssm.str().c_str());
  

  // gravity compensation
  //Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);  // Joint positions (set to zero or desired configuration)
  Eigen::Vector3d v = dq;//Eigen::VectorXd::Zero(pinocchio_model.nv);  // Joint velocities (zero for static case)  
  Eigen::Vector3d a = Eigen::VectorXd::Zero(pinocchio_model.nv); //ddq_d;// // Joint accelerations (zero for static case)

  Eigen::Vector3d tau_dynamics = pinocchio::rnea(pinocchio_model, pinocchio_data, q, v, a);
  tau = tau + tau_dynamics;
  // std::cout << "Gravity compensation torques: " << gravity_torques.transpose() << std::endl;

  //tau = 2.0/3.0 * (tau.array() +  gravity_torques.array());
  tau = 2.0/3.0 * tau.array();
// std::cout<<"tau COMBINED "<<tau(0)<<" "<<tau(1)<<" "<<tau(2)<<" "<<"\n"<<std::endl;
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    // if (!std::isnan((*current_ref)->displacements[i]))
    // {

      bool is_set_successfully = command_interfaces_[i].set_value(tau(i));
      if (!is_set_successfully){
          RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Failed to set_value in controller");
      } else {
        RCLCPP_INFO_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          (std::string("Successfully set new torque:")+std::to_string(tau(0))+std::string(" ")+std::to_string(tau(1))+std::string(" ")+std::to_string(tau(2))+std::string("\n")).c_str());
          
      }
      //(*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
    // }
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace padman_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  padman_controller::PadmanController, controller_interface::ControllerInterface)
