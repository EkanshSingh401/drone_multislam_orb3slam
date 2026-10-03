/**
 * @file stereo-inertial-slam-node.cpp
 * @brief Implementation of the StereoInertialSlamNode wrapper class.
 */
#include "stereo-inertial-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace ORB_SLAM3_Wrapper
{
    using namespace WrapperTypeConversions;

    StereoInertialSlamNode::StereoInertialSlamNode(const std::string &strVocFile,
                                                   const std::string &strSettingsFile,
                                                   ORB_SLAM3::System::eSensor sensor)
        : SlamNodeBase("ORB_SLAM3_STEREO_INERTIAL_ROS2", strVocFile, strSettingsFile, sensor)
    {
        // Defaults match the Jetson/D455 topic names so configs transfer.
        this->declare_parameter("left_image_topic_name",
                                rclcpp::ParameterValue("/camera/infra1/image_rect_raw"));
        this->declare_parameter("right_image_topic_name",
                                rclcpp::ParameterValue("/camera/infra2/image_rect_raw"));
        this->declare_parameter("imu_topic_name", rclcpp::ParameterValue("/camera/imu"));

        const auto leftTopic = this->get_parameter("left_image_topic_name").as_string();
        const auto rightTopic = this->get_parameter("right_image_topic_name").as_string();
        const auto imuTopic = this->get_parameter("imu_topic_name").as_string();

        leftSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, leftTopic);
        rightSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, rightTopic);
        syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
            approximate_sync_policy(10), *leftSub_, *rightSub_);
        syncApproximate_->registerCallback(&StereoInertialSlamNode::StereoCallback, this);

        imuCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions imuSubOptions;
        imuSubOptions.callback_group = imuCallbackGroup_;
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, rclcpp::SensorDataQoS(),
            std::bind(&StereoInertialSlamNode::ImuCallback, this, std::placeholders::_1),
            imuSubOptions);

        RCLCPP_INFO(this->get_logger(),
                    "STEREO-INERTIAL node started. left: %s | right: %s | imu: %s",
                    leftTopic.c_str(), rightTopic.c_str(), imuTopic.c_str());
    }

    StereoInertialSlamNode::~StereoInertialSlamNode()
    {
        leftSub_.reset();
        rightSub_.reset();
        syncApproximate_.reset();
        imuSub_.reset();
        RCLCPP_INFO(this->get_logger(),
                    "STEREO-INERTIAL node stopped. frames=%zu imu_samples=%zu "
                    "imu_initialised=%s",
                    frameCount_, imuSampleCount_, imuInitialised_ ? "yes" : "no");
    }

    void StereoInertialSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgImu)
    {
        std::lock_guard<std::mutex> lock(imuMutex_);
        imuBuf_.push(msgImu);
        ++imuSampleCount_;
    }

    void StereoInertialSlamNode::StereoCallback(const sensor_msgs::msg::Image::SharedPtr msgLeft,
                                                const sensor_msgs::msg::Image::SharedPtr msgRight)
    {
        cv_bridge::CvImageConstPtr cvLeft;
        cv_bridge::CvImageConstPtr cvRight;
        try
        {
            // The IR imagers are mono8, which cv_bridge hands back as CV_8UC1 --
            // exactly what ORB-SLAM3 wants, with no colour conversion.
            cvLeft = cv_bridge::toCvShare(msgLeft);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (left): %s", e.what());
            return;
        }
        try
        {
            cvRight = cv_bridge::toCvShare(msgRight);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (right): %s", e.what());
            return;
        }

        const double tFrame = stampToSec(msgLeft->header.stamp);

        // Drain every IMU sample up to this frame's timestamp. ORB-SLAM3
        // preintegrates the samples handed to it between the previous and the
        // current frame, so they must be contiguous and must not be reused --
        // hence popping rather than peeking.
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            std::lock_guard<std::mutex> lock(imuMutex_);
            while (!imuBuf_.empty() && stampToSec(imuBuf_.front()->header.stamp) <= tFrame)
            {
                const auto &m = imuBuf_.front();
                const double t = stampToSec(m->header.stamp);
                cv::Point3f acc(m->linear_acceleration.x, m->linear_acceleration.y,
                                m->linear_acceleration.z);
                cv::Point3f gyr(m->angular_velocity.x, m->angular_velocity.y,
                                m->angular_velocity.z);
                vImuMeas.emplace_back(acc, gyr, t);
                imuBuf_.pop();
            }
        }

        if (vImuMeas.empty())
        {
            // Normal exactly once, for the first frame (nothing precedes it).
            // Sustained warnings mean the IMU topic is absent or its timestamps
            // disagree with the images -- both fatal for stereo-inertial, so say
            // so rather than silently degrading to visual-only.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "No IMU measurements up to frame t=%.6f; "
                                 "stereo-inertial cannot preintegrate.", tFrame);
            return;
        }

        if (firstFrameStamp_ < 0.0)
        {
            firstFrameStamp_ = tFrame;
            RCLCPP_INFO(this->get_logger(),
                        "first stereo pair at t=%.6f with %zu IMU samples",
                        tFrame, vImuMeas.size());
        }
        ++frameCount_;
        lastFrameStamp_ = tFrame;

        auto Tcw = interface()->slam()->TrackStereo(cvLeft->image, cvRight->image,
                                                    tFrame, vImuMeas);

        // Report IMU initialisation exactly once. GetTimeFromIMUInit() returns
        // > 0 only after Atlas::isImuInitialized(), so it is a reliable edge.
        if (!imuInitialised_)
        {
            const double tFromInit = interface()->slam()->GetTimeFromIMUInit();
            if (tFromInit > 0.0)
            {
                imuInitialised_ = true;
                imuInitStamp_ = tFrame;
                RCLCPP_INFO(this->get_logger(),
                            "IMU INITIALISED at frame t=%.6f "
                            "(%.3f s after first frame, frame #%zu, %zu IMU samples seen)",
                            tFrame, tFrame - firstFrameStamp_, frameCount_, imuSampleCount_);
            }
        }

        if (interface()->processTrackedPose(Tcw))
        {
            this->onTracked(msgLeft->header);
        }
    }
} // namespace ORB_SLAM3_Wrapper
