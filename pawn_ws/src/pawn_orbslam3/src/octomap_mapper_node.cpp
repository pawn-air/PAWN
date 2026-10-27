#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace
{
struct VoxelIndex
{
    int x;
    int y;
    int z;

    bool operator==(const VoxelIndex &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelIndexHash
{
    std::size_t operator()(const VoxelIndex &k) const
    {
        std::size_t h1 = std::hash<int>{}(k.x);
        std::size_t h2 = std::hash<int>{}(k.y);
        std::size_t h3 = std::hash<int>{}(k.z);
        return h1 ^ (h2 * 0x9e3779b9 + (h1 << 6) + (h1 >> 2)) ^
               (h3 * 0x9e3779b9 + (h2 << 6) + (h2 >> 2));
    }
};
}  // namespace

/**
 * Kisi B - 3B Hacimsel Haritalama ve Engel Algilama Dugumu (OctomapMapperNode)
 *
 * Gorevleri:
 *  1. Faz 1 (Paket & Koordinat Uyumu + Otomatik Metrik Olcek Kalibrasyonu + Gurultu Filtreleme):
 *     Kisi A'dan gelen /drone_<id>/slam/map_points ve /drone_<id>/slam/pose verilerini dinler.
 *     Monokuler ORB-SLAM3'un baslangictaki kucuk titreme/yerinde hareketlerle (ornegin EuRoC MH02)
 *     haritayi devasa (keyfi) olcekte baslatmasi sorununu cozmek icin:
 *       - Otomatik Derinlik Kalibrasyonu (auto_scale): Ilk nokta bulutunun medyan kamera derinligini
 *         gercekci metrik oda derinligine (target_median_depth, orn. 2.5m) normalize eder.
 *       - Dinamik Kamera-Bagil Yukseklik Filtresi (relative_z_filter): Drone'un her yukseklikte ve
 *         egimde kesintisiz haritalama yapabilmesi icin Z sinirlarini drone'un o anki konumuna gore uygular.
 *  2. Faz 2 (3B Octree & Raycasting):
 *     Kamera konumundan (sensor origin) gorus acisindaki (FOV) temizlenmis noktalara isin izleme
 *     (raycasting) yaparak Bos (Free) ve Dolu (Occupied) kupleri (voxel) hesaplar.
 *  3. Faz 3 (Kontrol & Kesif Arayuzu):
 *     - En yakin engelin vektorunu (/drone_<id>/map/closest_obstacle) ve carpisma uyarisini
 *       (/drone_<id>/map/collision_warning) yayinlar.
 *     - Otonom kesif icin Bos-Bilinmeyen sinirindaki Frontier kuplerini cikarir ve hem
 *       /drone_<id>/map/frontiers hem de /autopilot/frontiers (MarkerArray) olarak yayinlar.
 *     - Global harita nokta bulutunu (/global_map/pointcloud) yayinlar.
 *  4. Faz 4 (Olcek Guncelleme & Sifirlama):
 *     /drone_<id>/slam/scale degistiginde veya /drone_<id>/map/reset servisi cagrildiginda
 *     Octree haritasini sifirlar.
 */
class OctomapMapperNode : public rclcpp::Node
{
public:
    OctomapMapperNode()
    : Node("octomap_mapper_node"), has_pose_(false)
    {
        // --- Temel & Multi-Drone Parametreleri ---
        this->declare_parameter<int>("drone_id", 1);
        this->declare_parameter<std::string>("map_frame", "");
        this->declare_parameter<std::string>("pose_topic", "");
        this->declare_parameter<std::string>("cloud_topic", "");
        this->declare_parameter<bool>("convert_optical_to_flu", false);
        this->declare_parameter<double>("initial_scale", 1.0);

        // --- Monokuler Olcek (Metric Scale) Kalibrasyon Parametreleri ---
        this->declare_parameter<bool>("auto_scale", true);            // Monokuler SLAM keyfi olcegini otomatik kalibre et
        this->declare_parameter<double>("target_median_depth", 2.50); // Ilk sahnenin beklenen medyan derinligi (m)
        this->declare_parameter<bool>("publish_scaled_pose", true);   // Olceklenmis pozu /drone_<id>/map/scaled_pose'da yayinla
        this->declare_parameter<bool>("align_with_slam_pose", false); // RViz'de Kisi A'nin ham /slam/pose'u ile ust uste otursun mu?

        // --- Octree Parametreleri ---
        this->declare_parameter<double>("resolution", 0.08);          // 8 cm kup boyutu (genis oda/salonlar icin ideal)
        this->declare_parameter<double>("prob_hit", 0.70);            // Dolu olma olasilik artisi
        this->declare_parameter<double>("prob_miss", 0.40);           // Bos olma olasilik dususu (raycasting)
        this->declare_parameter<double>("clamping_min", 0.12);
        this->declare_parameter<double>("clamping_max", 0.97);

        // --- Faz 1: Filtreleme Parametreleri ---
        this->declare_parameter<double>("min_range", 0.15);           // Kameraya < 15 cm noktalari ele
        this->declare_parameter<double>("max_range", 12.00);          // Kameraya > 12 m noktalari ele (genis odalar icin)
        this->declare_parameter<bool>("relative_z_filter", true);     // Z sinirini drone'un o anki yuksekligine gore uygula
        this->declare_parameter<double>("min_z", -6.00);              // Drone'a gore (veya mutlak) alt yukseklik siniri
        this->declare_parameter<double>("max_z", 6.00);               // Drone'a gore (veya mutlak) ust yukseklik siniri
        this->declare_parameter<double>("outlier_radius", 0.35);      // Komsuluk arama yaricapi (m)
        this->declare_parameter<int>("min_neighbors", 2);             // En az komsu nokta sayisi
        this->declare_parameter<bool>("use_fov_filter", false);       // Kamera arkasi filtreleme (genel kesifte tam harita icin kapali)

        // --- Faz 3: Kontrol & Kesif (Frontier) Parametreleri ---
        this->declare_parameter<double>("safety_check_radius", 1.50); // Drone etrafinda engel arama yaricapi (m)
        this->declare_parameter<double>("drone_safety_radius", 0.30); // Carpisma tehlike yaricapi (m)
        this->declare_parameter<double>("frontier_min_z", -4.00);     // Frontier arama alt yuksekligi (m)
        this->declare_parameter<double>("frontier_max_z", 4.00);      // Frontier arama ust yuksekligi (m)
        this->declare_parameter<double>("frontier_search_radius", 8.0); // Drone etrafinda frontier arama yaricapi (m)

        drone_id_ = this->get_parameter("drone_id").as_int();
        if (drone_id_ < 1) {
            throw std::runtime_error("drone_id must be >= 1");
        }

        const std::string prefix = "/drone_" + std::to_string(drone_id_);

        map_frame_ = this->get_parameter("map_frame").as_string();
        if (map_frame_.empty()) {
            map_frame_ = "drone_" + std::to_string(drone_id_) + "/slam_map";
        }

        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        if (pose_topic.empty()) {
            pose_topic = prefix + "/slam/pose";
        }

        std::string cloud_topic = this->get_parameter("cloud_topic").as_string();
        if (cloud_topic.empty()) {
            cloud_topic = prefix + "/slam/map_points";
        }

        convert_optical_to_flu_ = this->get_parameter("convert_optical_to_flu").as_bool();
        scale_ = static_cast<float>(this->get_parameter("initial_scale").as_double());
        auto_scale_ = this->get_parameter("auto_scale").as_bool();
        target_median_depth_ = static_cast<float>(this->get_parameter("target_median_depth").as_double());
        publish_scaled_pose_ = this->get_parameter("publish_scaled_pose").as_bool();
        align_with_slam_pose_ = this->get_parameter("align_with_slam_pose").as_bool();

        // Eger RViz'de Kisi A'nin ham /slam/pose ve /slam/map_points'i ile birebir ayni
        // koordinat uzayinda gorunmesi isteniyorsa, filtre esiklerini SLAM'in ic olcegine gore
        // dinamik olceklendirecegiz.
        scale_calibrated_ = !auto_scale_;

        resolution_ = this->get_parameter("resolution").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        relative_z_filter_ = this->get_parameter("relative_z_filter").as_bool();
        min_z_ = this->get_parameter("min_z").as_double();
        max_z_ = this->get_parameter("max_z").as_double();
        outlier_radius_ = this->get_parameter("outlier_radius").as_double();
        min_neighbors_ = this->get_parameter("min_neighbors").as_int();
        use_fov_filter_ = this->get_parameter("use_fov_filter").as_bool();
        safety_check_radius_ = this->get_parameter("safety_check_radius").as_double();
        drone_safety_radius_ = this->get_parameter("drone_safety_radius").as_double();
        frontier_min_z_ = this->get_parameter("frontier_min_z").as_double();
        frontier_max_z_ = this->get_parameter("frontier_max_z").as_double();
        frontier_search_radius_ = this->get_parameter("frontier_search_radius").as_double();

        // --- Octree Baslatma ---
        octree_ = std::make_unique<octomap::OcTree>(resolution_);
        octree_->setProbHit(this->get_parameter("prob_hit").as_double());
        octree_->setProbMiss(this->get_parameter("prob_miss").as_double());
        octree_->setClampingThresMin(this->get_parameter("clamping_min").as_double());
        octree_->setClampingThresMax(this->get_parameter("clamping_max").as_double());

        // --- Abonelikler (Kisi A'dan Gelen Veriler) ---
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10,
            std::bind(&OctomapMapperNode::poseCallback, this, std::placeholders::_1));

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, 10,
            std::bind(&OctomapMapperNode::cloudCallback, this, std::placeholders::_1));

        scale_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            prefix + "/slam/scale", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                if (msg->data > 0.0f && std::isfinite(msg->data)) {
                    // Kisi A (orb_slam_node) /slam/scale yayinladiginda, nokta bulutunu ve pozu
                    // zaten bu olcekle carparak yayinliyor. Bu yuzden Kisi B'de cift olcekleme
                    // yapmamak icin scale_ = 1.0f tutulur ve harita temizlenir.
                    scale_ = 1.0f;
                    scale_calibrated_ = true;
                    octree_->clear();
                    RCLCPP_INFO(this->get_logger(),
                        "SLAM dugumunden metrik olcek (%.4f) alindi -> Octree haritasi sifirlandi.", msg->data);
                }
            });

        // --- Yayincilar (Kontrol Ekibi, PyVista GUI ve RViz2 Icin Ciktilar) ---
        scaled_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            prefix + "/map/scaled_pose", 10);
        estimated_scale_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            prefix + "/map/estimated_scale", 10);
        filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            prefix + "/slam_points_filtered", 10);
        global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/global_map/pointcloud", 10);
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
            prefix + "/octomap_binary", 10);
        occupied_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            prefix + "/occupied_cells_vis_array", 10);
        closest_obstacle_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            prefix + "/map/closest_obstacle", 10);
        collision_warn_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            prefix + "/map/collision_warning", 10);
        frontier_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            prefix + "/map/frontiers", 10);
        frontier_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            prefix + "/map/frontier_markers", 10);
        autopilot_frontiers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/autopilot/frontiers", 10);

        // --- Faz 4: Harita Sifirlama Servisi ---
        reset_srv_ = this->create_service<std_srvs::srv::Empty>(
            prefix + "/map/reset",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>) {
                octree_->clear();
                scale_calibrated_ = !auto_scale_;
                RCLCPP_INFO(this->get_logger(), "Harita sifirlama servisi cagrildi -> Octree ve olcek temizlendi.");
            });

        RCLCPP_INFO(this->get_logger(),
            "OctomapMapperNode (Kisi B) baslatildi! Drone: %d | Frame: %s | Cozunurluk: %.2f m | AutoScale: %s",
            drone_id_, map_frame_.c_str(), resolution_, auto_scale_ ? "ON" : "OFF");
    }

private:
    // Koordinat donusumu yardimcisi (Optical [X-sag, Y-asagi, Z-ileri] -> FLU [X-ileri, Y-sol, Z-yukari])
    Eigen::Vector3f transformPoint(const Eigen::Vector3f &pt_in) const
    {
        const float s = align_with_slam_pose_ ? 1.0f : scale_;
        Eigen::Vector3f scaled = pt_in * s;
        if (!convert_optical_to_flu_) {
            return scaled;
        }
        return Eigen::Vector3f(scaled.z(), -scaled.x(), -scaled.y());
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        Eigen::Vector3f raw_pos(
            static_cast<float>(msg->pose.position.x),
            static_cast<float>(msg->pose.position.y),
            static_cast<float>(msg->pose.position.z));

        // SLAM reset (yeni harita) tespiti: Eger poz aniden merkeze sifirlandiysa haritayi ve olcegi yenile
        if (has_pose_ && last_raw_pose_.norm() > 2.0f && raw_pos.norm() < 0.05f) {
            RCLCPP_WARN(this->get_logger(),
                "ORB-SLAM3 harita sifirlamasi (Tracking Reset) algilandi -> Octree ve olcek yeniden kalibre ediliyor.");
            octree_->clear();
            if (auto_scale_) {
                scale_calibrated_ = false;
            }
        }
        last_raw_pose_ = raw_pos;
        current_pose_ = transformPoint(raw_pos);

        Eigen::Quaternionf q(
            static_cast<float>(msg->pose.orientation.w),
            static_cast<float>(msg->pose.orientation.x),
            static_cast<float>(msg->pose.orientation.y),
            static_cast<float>(msg->pose.orientation.z));
        q.normalize();

        // FLU cercevesinde ileri yon +X (1, 0, 0)
        if (convert_optical_to_flu_) {
            Eigen::Vector3f forward_opt = q * Eigen::Vector3f(0.0f, 0.0f, 1.0f);
            current_forward_ = Eigen::Vector3f(forward_opt.z(), -forward_opt.x(), -forward_opt.y()).normalized();
        } else {
            current_forward_ = (q * Eigen::Vector3f(1.0f, 0.0f, 0.0f)).normalized();
        }

        has_pose_ = true;

        if (publish_scaled_pose_) {
            geometry_msgs::msg::PoseStamped scaled_msg = *msg;
            scaled_msg.header.frame_id = map_frame_;
            scaled_msg.pose.position.x = current_pose_.x();
            scaled_msg.pose.position.y = current_pose_.y();
            scaled_msg.pose.position.z = current_pose_.z();
            scaled_pose_pub_->publish(scaled_msg);
        }

        // Drone hareket ettikce en yakin engeli ve carpisma durumunu Kontrol ekibine bildir
        publishObstacleClearance(msg->header.stamp);
    }

    // Monokuler ORB-SLAM3 baslangic olcegini sahne medyan derinliginden otomatik tahmin et
    void calibrateMonocularScale(const sensor_msgs::msg::PointCloud2 &msg)
    {
        std::vector<float> dists;
        dists.reserve(msg.width * msg.height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            Eigen::Vector3f p(*iter_x, *iter_y, *iter_z);
            if (!p.allFinite()) continue;
            float d = (p - last_raw_pose_).norm();
            if (d > 1e-4f && std::isfinite(d)) {
                dists.push_back(d);
            }
        }

        if (dists.size() < 20) {
            return;
        }

        std::nth_element(dists.begin(), dists.begin() + dists.size() / 2, dists.end());
        float median_slam_depth = dists[dists.size() / 2];

        // Eger medyan derinlik zaten hedef araliktaysa (orn. Kisi A zaten olceklemisse [1.0m - 6.0m]),
        // ekstra olcekleme yapma (scale = 1.0)
        if (median_slam_depth >= 1.0f && median_slam_depth <= 6.0f) {
            scale_ = 1.0f;
            scale_calibrated_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Gelen nokta bulutu zaten metrik olcekte (Medyan Derinlik: %.2f m) -> Ek olcekleme uygulanmadi (scale=1.0).",
                median_slam_depth);
            return;
        }

        if (median_slam_depth > 1e-3f) {
            scale_ = std::clamp(target_median_depth_ / median_slam_depth, 0.02f, 50.0f);
            scale_calibrated_ = true;
            current_pose_ = transformPoint(last_raw_pose_);

            std_msgs::msg::Float32 scale_msg;
            scale_msg.data = scale_;
            estimated_scale_pub_->publish(scale_msg);

            RCLCPP_INFO(this->get_logger(),
                "Monokuler SLAM Olcek Kalibrasyonu Tamamlandi! Ham Medyan Derinlik: %.3f -> Hedef: %.2f m | "
                "Hesaplanan Olcek Carpani (scale): %.4f",
                median_slam_depth, target_median_depth_, scale_);
        }
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!has_pose_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "SLAM pose henuz gelmedi, nokta bulutu islenmek icin bekleniyor...");
            return;
        }

        // Ilk gecerli harita geldiginde monokuler SLAM olcegini otomatik kalibre et
        if (!scale_calibrated_) {
            calibrateMonocularScale(*msg);
        }

        // Eger align_with_slam_pose=true ise noktalar ham SLAM uzayinda tutulur, filtre yaricaplari SLAM uzayina esnetilir
        const float metric_to_map = align_with_slam_pose_ ? (1.0f / std::max(scale_, 1e-3f)) : 1.0f;

        const float eff_min_range = static_cast<float>(min_range_) * metric_to_map;
        const float eff_max_range = static_cast<float>(max_range_) * metric_to_map;
        const float eff_min_z = static_cast<float>(min_z_) * metric_to_map;
        const float eff_max_z = static_cast<float>(max_z_) * metric_to_map;
        const float eff_outlier_radius = static_cast<float>(outlier_radius_) * metric_to_map;

        // 1. Faz 1: Noktalari oku, koordinat/olcek uygula, mesafe ve yukseklik filtresinden gecir
        std::vector<Eigen::Vector3f> candidates;
        candidates.reserve(msg->width * msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        const float min_r2 = eff_min_range * eff_min_range;
        const float max_r2 = eff_max_range * eff_max_range;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            Eigen::Vector3f raw_pt(*iter_x, *iter_y, *iter_z);
            if (!raw_pt.allFinite()) continue;

            Eigen::Vector3f pt = transformPoint(raw_pt);
            Eigen::Vector3f rel = pt - current_pose_;

            // Yukseklik (Z) filtresi: Drone'a bagil (relative) veya mutlak (absolute)
            float z_val = relative_z_filter_ ? rel.z() : pt.z();
            if (z_val < eff_min_z || z_val > eff_max_z) continue;

            // Kamera mesafesi (Range) filtresi
            float dist2 = rel.squaredNorm();
            if (dist2 < min_r2 || dist2 > max_r2) continue;

            // Istege bagli FOV Filtresi
            if (use_fov_filter_ && rel.dot(current_forward_) <= 0.0f) {
                continue;
            }

            candidates.push_back(pt);
        }

        if (candidates.empty()) return;

        // 2. Faz 1: Hizli 3B Voxel-Grid Komsuluk Filtresi (Radius Outlier Removal - O(N))
        std::vector<Eigen::Vector3f> filtered_points;
        filtered_points.reserve(candidates.size());

        if (min_neighbors_ <= 0 || eff_outlier_radius <= 0.0f) {
            filtered_points = std::move(candidates);
        } else {
            const float cell_size = eff_outlier_radius;
            const float inv_cell = 1.0f / cell_size;
            const float radius2 = cell_size * cell_size;

            std::unordered_map<VoxelIndex, std::vector<size_t>, VoxelIndexHash> grid;
            grid.reserve(candidates.size());

            for (size_t i = 0; i < candidates.size(); ++i) {
                VoxelIndex idx{
                    static_cast<int>(std::floor(candidates[i].x() * inv_cell)),
                    static_cast<int>(std::floor(candidates[i].y() * inv_cell)),
                    static_cast<int>(std::floor(candidates[i].z() * inv_cell))
                };
                grid[idx].push_back(i);
            }

            for (size_t i = 0; i < candidates.size(); ++i) {
                const auto &pt = candidates[i];
                VoxelIndex base{
                    static_cast<int>(std::floor(pt.x() * inv_cell)),
                    static_cast<int>(std::floor(pt.y() * inv_cell)),
                    static_cast<int>(std::floor(pt.z() * inv_cell))
                };

                int neighbors = 0;
                for (int dx = -1; dx <= 1 && neighbors < min_neighbors_; ++dx) {
                    for (int dy = -1; dy <= 1 && neighbors < min_neighbors_; ++dy) {
                        for (int dz = -1; dz <= 1 && neighbors < min_neighbors_; ++dz) {
                            auto it = grid.find(VoxelIndex{base.x + dx, base.y + dy, base.z + dz});
                            if (it == grid.end()) continue;
                            for (size_t j : it->second) {
                                if (i == j) continue;
                                if ((pt - candidates[j]).squaredNorm() <= radius2) {
                                    neighbors++;
                                    if (neighbors >= min_neighbors_) break;
                                }
                            }
                        }
                    }
                }

                if (neighbors >= min_neighbors_) {
                    filtered_points.push_back(pt);
                }
            }
        }

        if (filtered_points.empty()) return;

        // Temizlenmis nokta bulutunu yayinla (/drone_<id>/slam_points_filtered)
        publishPointCloud(filtered_cloud_pub_, filtered_points, msg->header.stamp);

        // 3. Faz 2: Octree Guncelleme ve Raycasting (Isin Izleme)
        octomap::Pointcloud octo_cloud;
        octo_cloud.reserve(filtered_points.size());
        for (const auto &pt : filtered_points) {
            octo_cloud.push_back(pt.x(), pt.y(), pt.z());
        }

        octomap::point3d sensor_origin(current_pose_.x(), current_pose_.y(), current_pose_.z());
        octree_->insertPointCloud(octo_cloud, sensor_origin, eff_max_range);
        octree_->updateInnerOccupancy();

        // 4. Harita Ciktilarini Yayinla (OctoMap Binary, RViz2 Kupleri, Global Pointcloud, En Yakin Engel ve Frontier'ler)
        publishOctomapAndMarkers(msg->header.stamp);
        publishObstacleClearance(msg->header.stamp);
        publishFrontiers(msg->header.stamp);
    }

    void publishPointCloud(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
        const std::vector<Eigen::Vector3f> &points,
        const builtin_interfaces::msg::Time &stamp)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = stamp;
        cloud_msg.header.frame_id = map_frame_;
        cloud_msg.height = 1;
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> out_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud_msg, "z");

        for (const auto &pt : points) {
            *out_x = pt.x();
            *out_y = pt.y();
            *out_z = pt.z();
            ++out_x; ++out_y; ++out_z;
        }
        cloud_msg.width = static_cast<uint32_t>(points.size());
        pub->publish(cloud_msg);
    }

    void publishOctomapAndMarkers(const builtin_interfaces::msg::Time &stamp)
    {
        // 1. Binary Octomap mesaji (Kontrol/Planlama ekibi icin)
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msg.header.stamp = stamp;
        octomap_msg.header.frame_id = map_frame_;
        if (octomap_msgs::binaryMapToMsg(*octree_, octomap_msg)) {
            octomap_pub_->publish(octomap_msg);
        }

        // 2. RViz2 MarkerArray (Yukseklige gore renklendirilmis dolu kupler) & Global Voxel PointCloud
        visualization_msgs::msg::Marker cubes;
        cubes.header.stamp = stamp;
        cubes.header.frame_id = map_frame_;
        cubes.ns = "occupied_voxels";
        cubes.id = 0;
        cubes.type = visualization_msgs::msg::Marker::CUBE_LIST;
        cubes.action = visualization_msgs::msg::Marker::ADD;
        cubes.scale.x = resolution_;
        cubes.scale.y = resolution_;
        cubes.scale.z = resolution_;
        cubes.pose.orientation.w = 1.0;

        std::vector<Eigen::Vector3f> occupied_centers;
        float z_low = std::numeric_limits<float>::max();
        float z_high = std::numeric_limits<float>::lowest();

        for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it)
        {
            if (octree_->isNodeOccupied(*it))
            {
                float x = static_cast<float>(it.getX());
                float y = static_cast<float>(it.getY());
                float z = static_cast<float>(it.getZ());
                occupied_centers.emplace_back(x, y, z);
                z_low = std::min(z_low, z);
                z_high = std::max(z_high, z);
            }
        }

        const float z_span = std::max(0.5f, z_high - z_low);
        cubes.points.reserve(occupied_centers.size());
        cubes.colors.reserve(occupied_centers.size());

        for (const auto &pt : occupied_centers)
        {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            cubes.points.push_back(p);

            // Dinamik min/max yukseklige gore renk gecisi (Mavi -> Yesil -> Kirmizi)
            std_msgs::msg::ColorRGBA color;
            float norm_z = std::clamp((pt.z() - z_low) / z_span, 0.0f, 1.0f);
            color.r = norm_z;
            color.g = 1.0f - std::abs(norm_z - 0.5f) * 2.0f;
            color.b = 1.0f - norm_z;
            color.a = 0.85f;
            cubes.colors.push_back(color);
        }

        if (!occupied_centers.empty()) {
            visualization_msgs::msg::MarkerArray marker_array;
            marker_array.markers.push_back(cubes);
            occupied_markers_pub_->publish(marker_array);

            // ARCHITECTURE_AND_RULES.md C.1: /global_map/pointcloud
            publishPointCloud(global_map_pub_, occupied_centers, stamp);
        }
    }

    // Faz 3: Drone etrafindaki en yakin engeli bul ve Kontrol ekibine yayinla
    void publishObstacleClearance(const builtin_interfaces::msg::Time &stamp)
    {
        if (!has_pose_) return;

        const float metric_to_map = align_with_slam_pose_ ? (1.0f / std::max(scale_, 1e-3f)) : 1.0f;
        const float r = static_cast<float>(safety_check_radius_) * metric_to_map;
        const float safe_r = static_cast<float>(drone_safety_radius_) * metric_to_map;

        octomap::point3d min_bbx(current_pose_.x() - r, current_pose_.y() - r, current_pose_.z() - r);
        octomap::point3d max_bbx(current_pose_.x() + r, current_pose_.y() + r, current_pose_.z() + r);

        float min_dist2 = std::numeric_limits<float>::max();
        Eigen::Vector3f closest_vec(0.0f, 0.0f, 0.0f);
        bool found_obstacle = false;

        for (auto it = octree_->begin_leafs_bbx(min_bbx, max_bbx), end = octree_->end_leafs_bbx();
             it != end; ++it)
        {
            if (octree_->isNodeOccupied(*it))
            {
                Eigen::Vector3f voxel_pos(
                    static_cast<float>(it.getX()),
                    static_cast<float>(it.getY()),
                    static_cast<float>(it.getZ()));
                Eigen::Vector3f diff = voxel_pos - current_pose_;
                float d2 = diff.squaredNorm();
                if (d2 < min_dist2) {
                    min_dist2 = d2;
                    closest_vec = diff;
                    found_obstacle = true;
                }
            }
        }

        if (found_obstacle) {
            // Kontrol ekibine her zaman metrik (metre) cinsinden engel vektoru gonder
            Eigen::Vector3f metric_vec = align_with_slam_pose_ ? (closest_vec * scale_) : closest_vec;
            geometry_msgs::msg::Vector3Stamped vec_msg;
            vec_msg.header.stamp = stamp;
            vec_msg.header.frame_id = map_frame_;
            vec_msg.vector.x = metric_vec.x();
            vec_msg.vector.y = metric_vec.y();
            vec_msg.vector.z = metric_vec.z();
            closest_obstacle_pub_->publish(vec_msg);
        }

        std_msgs::msg::Bool warn_msg;
        warn_msg.data = found_obstacle && (min_dist2 <= (safe_r * safe_r));
        collision_warn_pub_->publish(warn_msg);
    }

    // Faz 3 & 4: Bos (Free) ve Bilinmeyen (Unknown) sinirindaki Frontier noktalarini cikar
    void publishFrontiers(const builtin_interfaces::msg::Time &stamp)
    {
        std::vector<Eigen::Vector3f> frontiers;
        const double step = resolution_;
        const double dirs[6][3] = {
            { step, 0.0, 0.0}, {-step, 0.0, 0.0},
            {0.0,  step, 0.0}, {0.0, -step, 0.0},
            {0.0, 0.0,  step}, {0.0, 0.0, -step}
        };

        const float metric_to_map = align_with_slam_pose_ ? (1.0f / std::max(scale_, 1e-3f)) : 1.0f;
        const float search_r = static_cast<float>(frontier_search_radius_) * metric_to_map;
        const float f_min_z = static_cast<float>(frontier_min_z_) * metric_to_map;
        const float f_max_z = static_cast<float>(frontier_max_z_) * metric_to_map;

        octomap::point3d min_bbx(
            current_pose_.x() - search_r,
            current_pose_.y() - search_r,
            current_pose_.z() + f_min_z);
        octomap::point3d max_bbx(
            current_pose_.x() + search_r,
            current_pose_.y() + search_r,
            current_pose_.z() + f_max_z);

        for (auto it = octree_->begin_leafs_bbx(min_bbx, max_bbx), end = octree_->end_leafs_bbx();
             it != end; ++it)
        {
            // Yalnizca Bos (Free) olan kupleri incele
            if (octree_->isNodeOccupied(*it)) continue;

            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            // 6 komsudan en az biri Bilinmeyen (NULL) ise bu kup bir Frontier'dir
            bool is_frontier = false;
            for (int d = 0; d < 6; ++d) {
                octomap::OcTreeNode* neighbor = octree_->search(
                    x + dirs[d][0], y + dirs[d][1], z + dirs[d][2]);
                if (neighbor == nullptr) {
                    is_frontier = true;
                    break;
                }
            }

            if (is_frontier) {
                frontiers.emplace_back(
                    static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
            }
        }

        if (frontiers.empty()) return;

        publishPointCloud(frontier_cloud_pub_, frontiers, stamp);

        visualization_msgs::msg::Marker frontier_marker;
        frontier_marker.header.stamp = stamp;
        frontier_marker.header.frame_id = map_frame_;
        frontier_marker.ns = "frontiers";
        frontier_marker.id = 0;
        frontier_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        frontier_marker.action = visualization_msgs::msg::Marker::ADD;
        frontier_marker.scale.x = resolution_ * 1.2;
        frontier_marker.scale.y = resolution_ * 1.2;
        frontier_marker.scale.z = resolution_ * 1.2;
        frontier_marker.pose.orientation.w = 1.0;
        frontier_marker.color.r = 1.0f;
        frontier_marker.color.g = 0.85f;
        frontier_marker.color.b = 0.0f;
        frontier_marker.color.a = 0.80f;

        for (const auto &pt : frontiers) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            frontier_marker.points.push_back(p);
        }
        frontier_marker_pub_->publish(frontier_marker);

        // ARCHITECTURE_AND_RULES.md D.2: /autopilot/frontiers (MarkerArray)
        visualization_msgs::msg::MarkerArray frontier_array;
        frontier_array.markers.push_back(frontier_marker);
        autopilot_frontiers_pub_->publish(frontier_array);
    }

    int drone_id_ = 1;
    std::unique_ptr<octomap::OcTree> octree_;
    std::string map_frame_;
    bool convert_optical_to_flu_ = false;
    float scale_ = 1.0f;
    bool auto_scale_ = true;
    bool scale_calibrated_ = false;
    float target_median_depth_ = 2.5f;
    bool publish_scaled_pose_ = true;
    bool align_with_slam_pose_ = false;

    double resolution_;
    double min_range_;
    double max_range_;
    bool relative_z_filter_ = true;
    double min_z_;
    double max_z_;
    double outlier_radius_;
    int min_neighbors_;
    bool use_fov_filter_ = false;
    double safety_check_radius_;
    double drone_safety_radius_;
    double frontier_min_z_;
    double frontier_max_z_;
    double frontier_search_radius_ = 8.0;

    Eigen::Vector3f last_raw_pose_{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f current_pose_{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f current_forward_{0.0f, 0.0f, 1.0f};
    bool has_pose_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr scale_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr scaled_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr estimated_scale_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occupied_markers_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr closest_obstacle_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_warn_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr autopilot_frontiers_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapMapperNode>());
    rclcpp::shutdown();
    return 0;
}
