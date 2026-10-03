/**
 * @file stereo-inertial-slam-node.hpp
 * @brief ORB-SLAM3 STEREO-INERTIAL (System::IMU_STEREO) ROS 2 node.
 *
 * The wrapper shipped rgbd, rgbd_imu, mono_imu and stereo nodes but no
 * stereo-inertial one, even though ORB-SLAM3 supports the mode. This composes
 * the two existing patterns: the stereo node's message_filters pair sync, and
 * the rgbd-imu node's buffered IMU drain.
 */

#ifndef STEREO_INERTIAL_SLAM_NODE_HPP_
#define STEREO_INERTIAL_SLAM_NODE_HPP_

#include <mutex>
#include <queue>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "orb_slam3_ros2_wrapper/slam_node_base.hpp"

namespace ORB_SLAM3_Wrapper
{
    class StereoInertialSlamNode : public SlamNodeBase
    {
    public:
        StereoInertialSlamNode(const std::string &strVocFile,
                               const std::string &strSettingsFile,
                               ORB_SLAM3::System::eSensor sensor);
        ~StereoInertialSlamNode();

    private:
        using approximate_sync_policy =
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                            sensor_msgs::msg::Image>;

        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgImu);
        void StereoCallback(const sensor_msgs::msg::Image::SharedPtr msgLeft,
                            const sensor_msgs::msg::Image::SharedPtr msgRight);

        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> leftSub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rightSub_;
        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate_;

        // IMU on its own callback group so a slow stereo callback cannot stall
        // IMU intake; dropping IMU samples breaks preintegration.
        rclcpp::CallbackGroup::SharedPtr imuCallbackGroup_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
        std::mutex imuMutex_;
        std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;

        // IMU-initialisation reporting. ORB-SLAM3 needs motion before it can
        // initialise the IMU (scale, gravity direction and biases), and until it
        // does the estimate is effectively visual-only -- so a run has to be
        // reported with and without that prefix.
        bool imuInitialised_{false};
        double firstFrameStamp_{-1.0};
        double imuInitStamp_{-1.0};
        std::size_t frameCount_{0};
        std::size_t imuSampleCount_{0};
        double lastFrameStamp_{-1.0};
        // Sim time of the last wall<->sim clock beacon; see StereoCallback.
        double lastBeaconStamp_{-1e9};
    };
} // namespace ORB_SLAM3_Wrapper

#endif // STEREO_INERTIAL_SLAM_NODE_HPP_
