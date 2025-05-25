
#include <rclcpp/rclcpp.hpp>
#include "impedance_controller/impedance_controller.hpp"

namespace impedance_controller {

void PoseInputServer::setpose(const std::shared_ptr<franka_msgs::srv::SetPose::Request> request,
                                std::shared_ptr<franka_msgs::srv::SetPose::Response> response){
                                    (*position_d_target_)[0] = request->x;
                                    (*position_d_target_)[1] = request->y;
                                    (*position_d_target_)[2] = request->z;
                                    (*rotation_d_target_)[0] = request->roll;
                                    (*rotation_d_target_)[1] = request->pitch;
                                    (*rotation_d_target_)[2] = request->yaw;

                                    response->success = true;
                                }

int PoseInputServer::main(int /*argc*/, char** /***argv*/){
    // rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_pose_server");

    rclcpp::Service<franka_msgs::srv::SetPose>::SharedPtr service = 
        node->create_service<franka_msgs::srv::SetPose>("set_pose",
                                                        std::bind(&PoseInputServer::setpose, this, std::placeholders::_1, std::placeholders::_2));
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
} //namespace impedance_controller