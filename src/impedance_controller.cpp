#include <impedance_controller/impedance_controller.hpp>
#include <impedance_controller/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <chrono>

using namespace std::chrono_literals;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace impedance_controller {

controller_interface::InterfaceConfiguration
ImpedanceController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type  = controller_interface::interface_configuration_type::INDIVIDUAL;
    for(int i=1; i <= num_joints_; i++){
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
    return config;
}

controller_interface::InterfaceConfiguration
ImpedanceController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = franka_cartesian_pose_->get_state_interface_names();
    for (int i = 1; i <= num_joints_; ++i) {
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    }
    for (int i = 1; i <= num_joints_; ++i) {
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    }
    for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
        config.names.push_back(franka_robot_model_name);
    }
    return config;
}

void ImpedanceController::update_parameters() {
    for (auto i = 0; i < num_joints_; ++i) {
      // TODO(yazi_ba) Can we get the state from its name?
      const auto& position_interface = state_interfaces_.at(16 + i);
      const auto& velocity_interface = state_interfaces_.at(23 + i);
      joint_positions_current_[i] = position_interface.get_value();
      joint_velocities_current_[i] = velocity_interface.get_value();
    }
  }

Eigen::Vector3d ImpedanceController::compute_new_position() {
    elapsed_time_ = elapsed_time_ + trajectory_period_;
    double radius = 0.1;

    double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_));

    double delta_x = radius * std::sin(angle);
    double delta_z = radius * (std::cos(angle) - 1);

    Eigen::Vector3d new_position = position_;
    new_position.x() -= delta_x;
    new_position.z() -= delta_z;
    new_position.x() = position_d_target_(0);
    new_position.z() = position_d_target_(2);

    return new_position;

}

Eigen::VectorXd ImpedanceController::compute_torque(const Eigen::Vector3d& new_position,
                                                    const Eigen::Matrix<double, 6, 7>& jacobian, 
                                                    const Eigen::Matrix<double, 7, 1>& coriolis,
                                                    const Eigen::Vector3d& orientation_error) {
    
    Vector7d joint_velocities_current_eigen(joint_velocities_current_.data());

    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current_eigen;

    Eigen::Matrix<double, 6, 1> error;
    error.setZero();
    error.head(3) << new_position - position_;

    error.tail(3) << orientation_error;

    // RCLCPP_INFO(get_node()->get_logger(), "Error: [%f, %f, %f]", error(0), error(1), error(2));

    Eigen::VectorXd tau_task = jacobian.transpose() * (stiffness_ * error - damping_ * (jacobian * dq_filtered_));
    Eigen::VectorXd tau_d = tau_task + coriolis;

    return tau_d;

}

CallbackReturn ImpedanceController::on_init(){

    PoseInputServer pose_server_obj(&position_d_target_, &rotation_d_target_);
    std::thread input_thread(&PoseInputServer::main, pose_server_obj, 0, nullptr);
    input_thread.detach();

    franka_cartesian_pose_ = std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>
                                (franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated));

    return CallbackReturn::SUCCESS;
}

bool ImpedanceController::assign_parameters(){
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    return true;
    
}

CallbackReturn ImpedanceController::on_configure(const rclcpp_lifecycle::State& ) {

    if(!assign_parameters()) {
        return CallbackReturn::FAILURE;
    }

    auto translational_stiffness = get_node()->get_parameter("translational_stiffness").as_double();
    auto rotational_stiffness = get_node()->get_parameter("rotational_stiffness").as_double();

    translational_stiffness_ = translational_stiffness;
    rotational_stiffness_ = rotational_stiffness;

    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name, 
                                                    arm_id_ + "/" + k_robot_state_interface_name));
    
    auto collision_client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>("/service_server/set_full_collision_behavior");
    
    while (!collision_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_node()->get_logger(), "service not available, waiting again...");
        }
    
        auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
        auto future_result = collision_client->async_send_request(request);
    
        auto success = future_result.get();
    
        if (!success->success) {
        RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
        return CallbackReturn::ERROR;
        } else {
        RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
        }
    
    
    
    return CallbackReturn::SUCCESS;

}

CallbackReturn ImpedanceController::on_activate(const rclcpp_lifecycle::State& ){


    initialization_flag_ = true;
    elapsed_time_ = 0.0;
    dq_filtered_.setZero();

    // Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    stiffness_.setZero();
    stiffness_.topLeftCorner(3, 3) << translational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);
    stiffness_.bottomRightCorner(3, 3) << rotational_stiffness_ * Eigen::MatrixXd::Identity(3, 3);
    damping_.setZero();
    damping_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness_) *
                                        Eigen::MatrixXd::Identity(3, 3);
    damping_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness_) *
                                            Eigen::MatrixXd::Identity(3, 3);

    franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  
    return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceController::on_deactivate(const rclcpp_lifecycle::State&){
    franka_cartesian_pose_->release_interfaces();
    return CallbackReturn::SUCCESS;
}


controller_interface::return_type ImpedanceController::update(const rclcpp::Time& , const rclcpp::Duration&){

    if (initialization_flag_) {
        std::tie(orientation_d, position_) =
            franka_cartesian_pose_->getInitialOrientationAndTranslation();

        initialization_flag_ = false;
      }

    // RCLCPP_INFO(get_node()->get_logger(), "position: [%f, %f]", position_d_target_(0), position_d_target_(2));

    std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
    std::array<double, 42> endeffector_jacobian_wrt_base =
    franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 16> pose_ = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);


    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(endeffector_jacobian_wrt_base.data());
    Eigen::Map<const Eigen::Matrix<double, 4, 4>> pose(pose_.data());
    position_ = pose.block<3, 1>(0, 3);

    Eigen::Matrix3d rotation_matrix = pose.block<3, 3>(0, 0);
    orientation_ = Eigen::Quaterniond(rotation_matrix);

    if(orientation_d.coeffs().dot(orientation_.coeffs()) < 0.0){
        orientation_.coeffs() << -orientation_.coeffs();
    }

    Eigen::Quaterniond error_quaternion(orientation_.inverse() * orientation_d);
    Eigen::Vector3d orientation_error = rotation_matrix * Eigen::Vector3d(
        error_quaternion.x(),
        error_quaternion.y(),
        error_quaternion.z()
    );

    update_parameters();

    Eigen::Vector3d new_position = compute_new_position();

    Eigen::VectorXd tau_d = compute_torque(new_position, jacobian, coriolis, orientation_error);

    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

    for (int i = 0; i < num_joints_; i++) {
        command_interfaces_[i].set_value(tau_d(i));
      }
      return controller_interface::return_type::OK;

}

}//namespace impedance_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(impedance_controller::ImpedanceController,
                       controller_interface::ControllerInterface)
