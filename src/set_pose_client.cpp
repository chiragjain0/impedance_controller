#include "rclcpp/rclcpp.hpp"
#include <franka_msgs/srv/set_pose.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class SetPoseClient : public rclcpp::Node {
public:
    SetPoseClient() : Node("set_pose_client") {
        // Create a client for the "set_pose" service
        client_ = this->create_client<franka_msgs::srv::SetPose>("set_pose");

        // Wait for the service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        // Call the service asynchronously after waiting for it to become available
        send_request();
    }

private:
    void send_request() {
        // Create the request message
        auto request = std::make_shared<franka_msgs::srv::SetPose::Request>();

        // Populate the request with the desired pose
        request->x = 1.0;
        request->y = 2.0;
        request->z = 3.0;
        request->roll = 0.0;
        request->pitch = 0.0;
        request->yaw = 0.0;

        // Send the request and wait for the response
        auto future = client_->async_send_request(request);

        // Wait for the result (blocking)
        try {
            auto response = future.get();  // Wait for the response
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Pose set successfully!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to set pose.");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    rclcpp::Client<franka_msgs::srv::SetPose>::SharedPtr client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetPoseClient>());
    rclcpp::shutdown();
    return 0;
}
