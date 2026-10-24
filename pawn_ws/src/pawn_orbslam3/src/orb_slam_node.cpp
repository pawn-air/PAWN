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
#include <cv_bridge/cv_bridge.hpp>

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
                return;
            }

            // 4. Camera optical pose in arbitrary-scale local SLAM coordinates
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f position = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();

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
            *iter_x = p.x();
            *iter_y = p.y();
            *iter_z = p.z();

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        cloud_pub_->publish(cloud);

        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(
                get_logger(),
                "Full map published: %zu points",
                points.size()
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

    nav_msgs::msg::Path path_;
    std::string local_frame_;
    bool publish_path_ = true;

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
