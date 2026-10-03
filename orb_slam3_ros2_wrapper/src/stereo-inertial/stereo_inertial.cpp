#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-slam-node.hpp"

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orb_slam3_ros2_wrapper stereo_inertial "
                     "path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<ORB_SLAM3_Wrapper::StereoInertialSlamNode>(
        argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO);

    // MultiThreaded so the IMU callback group runs independently of the stereo
    // callback; a single-threaded executor would serialise them and drop IMU
    // samples while a stereo frame is being tracked.
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();

    return 0;
}
