#include <franka_msgs/srv/set_pose.hpp>


using namespace impedance_controller{

    class PoseInputServer {

        public :
        PoseInputServer (Eigen::Vector3d* position,
                        Eigen::Vector3d* rotation):
        position_d_target_(position),
        rotation_d_target_(rotation){}
        int main(int argc, char **argv);


        private:
            Eigen::Vector3d* position_d_target_;
            Eigen::Vector3d* rotation_d_target_;

            void setpose(const std::shared_ptr<franka_msgs::Srv::SetPose::Request> request,
                         std::shared_ptr<franka_msgs::srv::SetPose::Response> response);
    }

}