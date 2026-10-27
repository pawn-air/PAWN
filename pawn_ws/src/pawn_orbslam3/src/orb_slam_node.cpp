#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <functional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <algorithm>

#include <opencv2/core.hpp>
#include <Eigen/Geometry>

#include <System.h>
#include <MapPoint.h>

class PawnOrbSlamNode : public rclcpp::Node
{
public:
    PawnOrbSlamNode()
        : Node("pawn_orbslam3")
    {
        declare_parameter<std::string>(
            "vocabulary_path",
            ""
        );

        declare_parameter<std::string>(
            "settings_path",
            ""
        );

        declare_parameter<int>("drone_id", 1);
        declare_parameter<bool>("publish_path", true);
        publish_path_ = get_parameter("publish_path").as_bool();
        declare_parameter<std::string>("image_topic", "");
        declare_parameter<bool>("convert_optical_to_flu", true);
        declare_parameter<bool>("auto_metric_scale", true);
        declare_parameter<double>("reference_scene_depth", 2.5);
        convert_optical_to_flu_ = get_parameter("convert_optical_to_flu").as_bool();
        auto_metric_scale_ = get_parameter("auto_metric_scale").as_bool();
        reference_scene_depth_ = static_cast<float>(get_parameter("reference_scene_depth").as_double());

        const auto vocabulary =
            get_parameter("vocabulary_path").as_string();

        const auto settings =
            get_parameter("settings_path").as_string();

        const int drone_id = get_parameter("drone_id").as_int();
        if (drone_id < 1) {
            throw std::runtime_error("drone_id must be >= 1");
        }

        const std::string prefix = "/drone_" + std::to_string(drone_id);

        const auto configured_image_topic =
            get_parameter("image_topic").as_string();

        const std::string image_topic = configured_image_topic.empty()
            ? prefix + "/camera/image_raw"
            : configured_image_topic;

        RCLCPP_INFO(get_logger(), "Initializing ORB-SLAM3...");

        slam_ = std::make_unique<ORB_SLAM3::System>(
            vocabulary,
            settings,
            ORB_SLAM3::System::MONOCULAR,
            false
        );

        pose_pub_ =
            create_publisher<geometry_msgs::msg::PoseStamped>(
                prefix + "/slam/pose", 10
            );

        path_pub_ =
            create_publisher<nav_msgs::msg::Path>(
                prefix + "/path", 10
            );

        cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                prefix + "/slam/map_points", 10
            );

        scale_pub_ =
            create_publisher<std_msgs::msg::Float32>(
                prefix + "/slam/scale", 10
            );

        local_frame_ = "drone_" + std::to_string(drone_id) + "/slam_map";
        path_.header.frame_id = local_frame_;

        image_sub_ =
            create_subscription<sensor_msgs::msg::Image>(
                image_topic,
                rclcpp::SensorDataQoS(),
                std::bind(
                    &PawnOrbSlamNode::imageCallback,
                    this,
                    std::placeholders::_1
                )
            );

        RCLCPP_INFO(
            get_logger(),
            "PAWN SLAM ready. Waiting for camera images..."
        );
    }

    ~PawnOrbSlamNode() override
    {
        if (slam_) {
            slam_->Shutdown();
        }
    }

private:

    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try {
            // 1. ROS Image -> OpenCV
            auto cv_ptr = cv_bridge::toCvCopy(
                msg, "mono8"
            );

            // 2. Timestamp in seconds
            double timestamp =
                static_cast<double>(msg->header.stamp.sec) +
                static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

            // 3. Run SLAM
            Sophus::SE3f Tcw =
                slam_->TrackMonocular(
                    cv_ptr->image,
                    timestamp
                );

            // Tracking diagnostics
            image_count_++;
            const int state = slam_->GetTrackingState();

            if (state != last_tracking_state_ || image_count_ % 30 == 0) {
                RCLCPP_INFO(
                    get_logger(),
                    "Images: %zu | Tracking state: %d | Timestamp: %.3f",
                    image_count_, state, timestamp
                );
                last_tracking_state_ = state;
            }

            // Publish poses only when tracking is successful
            if (state != 2) {
                if (state == 3) {
                    // Tracking lost/reset -> recalibrate scale on recovery
                    scale_initialized_ = false;
                }
                return;
            }

            // 4. Camera optical pose in local SLAM coordinates
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f raw_position = Twc.translation();

            // Detect map reset if camera jumps back near origin after moving far
            if (scale_initialized_ && last_raw_position_.norm() > 1.5f && raw_position.norm() < 0.05f) {
                scale_initialized_ = false;
                path_.poses.clear();
                RCLCPP_WARN(get_logger(), "SLAM map reset detected -> recalibrating metric scale...");
            }
            last_raw_position_ = raw_position;

            // Calibrate monocular scale once initial map points are available
            if (auto_metric_scale_ && !scale_initialized_) {
                calibrateMetricScale(raw_position);
            }

            Eigen::Vector3f scaled_pos = raw_position * metric_scale_;
            Eigen::Vector3f position = opticalToFluPosition(scaled_pos);
            Eigen::Quaternionf q = convert_optical_to_flu_
                ? opticalToFluOrientation(Twc.rotationMatrix())
                : Twc.unit_quaternion();

            geometry_msgs::msg::PoseStamped pose;

            pose.header.stamp = msg->header.stamp;
            pose.header.frame_id = local_frame_;

            pose.pose.position.x = position.x();
            pose.pose.position.y = position.y();
            pose.pose.position.z = position.z();

            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            // Publish current pose
            pose_pub_->publish(pose);

            frame_count_++;

            // 5. Update trajectory
            path_.header.stamp = msg->header.stamp;
            if (publish_path_) {
                path_.poses.push_back(pose);
            }

            // Publish Path and PointCloud every 5 tracked frames
            if (frame_count_ % 5 == 0) {
                if (publish_path_) {
                    path_pub_->publish(path_);
                }
                publishMapPoints(msg->header.stamp);
            }

            if (frame_count_ % 100 == 0) {
                RCLCPP_INFO(
                    get_logger(),
                    "Tracked frames: %zu | Path poses: %zu",
                    frame_count_,
                    path_.poses.size()
                );
            }
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(
                get_logger(),
                "SLAM error: %s",
                e.what()
            );
        }
    }


    void calibrateMetricScale(const Eigen::Vector3f &raw_camera_pos)
    {
        auto points = slam_->GetAllMapPointPositions();
        if (points.size() < 25) {
            return;
        }

        std::vector<float> dists;
        dists.reserve(points.size());
        for (const auto &p : points) {
            if (!p.allFinite()) continue;
            float d = (p - raw_camera_pos).norm();
            if (d > 1e-4f && std::isfinite(d)) {
                dists.push_back(d);
            }
        }

        if (dists.size() < 20) {
            return;
        }

        std::nth_element(dists.begin(), dists.begin() + dists.size() / 2, dists.end());
        float median_depth = dists[dists.size() / 2];

        if (median_depth > 1e-3f) {
            metric_scale_ = std::clamp(reference_scene_depth_ / median_depth, 0.02f, 50.0f);
            scale_initialized_ = true;
            path_.poses.clear();

            std_msgs::msg::Float32 scale_msg;
            scale_msg.data = metric_scale_;
            scale_pub_->publish(scale_msg);

            RCLCPP_INFO(
                get_logger(),
                "Monocular scale calibrated: raw median depth=%.3f -> target=%.2f m (metric_scale=%.4f)",
                median_depth,
                reference_scene_depth_,
                metric_scale_
            );
        }
    }

    Eigen::Vector3f opticalToFluPosition(const Eigen::Vector3f &opt) const
    {
        if (!convert_optical_to_flu_) {
            return opt;
        }
        // Optical (X: right, Y: down, Z: forward) -> ROS FLU (X: forward, Y: left, Z: up)
        return Eigen::Vector3f(opt.z(), -opt.x(), -opt.y());
    }

    Eigen::Quaternionf opticalToFluOrientation(const Eigen::Matrix3f &R_wc_opt) const
    {
        // R_flu_opt transforms vectors from optical frame to FLU frame:
        // [x_flu; y_flu; z_flu] = [0 0 1; -1 0 0; 0 -1 0] * [x_opt; y_opt; z_opt]
        Eigen::Matrix3f R_flu_opt;
        R_flu_opt <<  0.0f,  0.0f, 1.0f,
                     -1.0f,  0.0f, 0.0f,
                      0.0f, -1.0f, 0.0f;
        Eigen::Matrix3f R_body_flu = R_flu_opt * R_wc_opt * R_flu_opt.transpose();
        Eigen::Quaternionf q(R_body_flu);
        q.normalize();
        return q;
    }

    void publishMapPoints(
        const builtin_interfaces::msg::Time &stamp)
    {
        // Full active SLAM map
        auto points = slam_->GetAllMapPointPositions();

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = local_frame_;
        cloud.header.stamp = stamp;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto &p : points) {
            Eigen::Vector3f pt = opticalToFluPosition(p * metric_scale_);
            *iter_x = pt.x();
            *iter_y = pt.y();
            *iter_z = pt.z();

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        cloud_pub_->publish(cloud);

        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(
                get_logger(),
                "Full map published: %zu points (scale=%.3f)",
                points.size(),
                metric_scale_
            );
        }
    }

    std::unique_ptr<ORB_SLAM3::System> slam_;

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr image_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    rclcpp::Publisher<
        nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Float32>::SharedPtr scale_pub_;

    nav_msgs::msg::Path path_;
    std::string local_frame_;
    bool publish_path_ = true;
    bool convert_optical_to_flu_ = true;
    bool auto_metric_scale_ = true;
    bool scale_initialized_ = false;
    float reference_scene_depth_ = 2.5f;
    float metric_scale_ = 1.0f;
    Eigen::Vector3f last_raw_position_{0.0f, 0.0f, 0.0f};

    size_t frame_count_ = 0;
    size_t image_count_ = 0;
    int last_tracking_state_ = -1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    {
        auto node = std::make_shared<PawnOrbSlamNode>();
        rclcpp::spin(node);
    }

    rclcpp::shutdown();

    return 0;
}
