#pragma once

#include <string>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace impedance_controller {

class ImpedanceController : public controller_interface::ControllerInterface {

    public :
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;


    private :

    bool assign_parameters();
    void update_joint_states();

    Eigen::Vector3d compute_new_position();
    
    std::vector<double> joint_positions_current_{0, 0, 0, 0, 0, 0, 0};
    std::vector<double> joint_velocities_current_{0, 0, 0, 0, 0, 0, 0};

    std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

    std::string arm_id_;
    int num_joints_{7};
    const bool k_elbow_activated{false};
    Vector7d k_gains_;
    Vector7d d_gains_;
    double elapsed_time_{0.0};
    double trajectory_period_{0.001};

    Eigen::Quaterniond orientation_;
    Eigen::Vector3d position_;

    bool initialization_flag_;

    Eigen::Matrix<double, 6, 6> stiffness_;
    Eigen::Matrix<double, 6, 6> damping_;
  

    const std::string k_robot_model_interface_name{"robot_model"};
    const std::string k_robot_state_interface_name{"robot_state"};

  // Compliance parameters
  const double translational_stiffness{1050.0};
  const double rotational_stiffness{10.0};



};

}// namespace impedance_controller