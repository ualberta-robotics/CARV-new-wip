#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <quest3carv_interfaces/msg/keyframe_data.hpp>

#include <Eigen/Dense>
#include "quest3carv_cpp/FreespaceDelaunayAlgorithm.h"
#include <unordered_map>
#include <vector>
#include <cmath>

// Voxel Key for 3D Deduplication
struct VoxelKey {
    int x, y, z;
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Custom Hash for VoxelKey
struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const {
        std::size_t h = 0;
        // Simple hash_combine logic
        h ^= std::hash<int>{}(k.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

class CarvingNode : public rclcpp::Node {
public:
    CarvingNode() : Node("carving_node"), keyframe_count_(0), process_every_n_frames_(1) {
        sub_kf_ = this->create_subscription<quest3carv_interfaces::msg::KeyframeData>(
            "quest3carv/keyframe", 10, 
            std::bind(&CarvingNode::keyframe_callback, this, std::placeholders::_1));

        pub_mesh_ = this->create_publisher<visualization_msgs::msg::Marker>("quest3carv/carved_mesh", 10);
        pub_points_ = this->create_publisher<visualization_msgs::msg::Marker>("quest3carv/carved_points", 10);

        RCLCPP_INFO(this->get_logger(), "Incremental Freespace Carving Node Initialized.");
    }

private:
    void keyframe_callback(const quest3carv_interfaces::msg::KeyframeData::SharedPtr msg) {
        keyframe_count_++;

        try {
            // 1. Extract Camera Optic Center
            Eigen::Vector3d cam_center(
                msg->camera_pose.position.x,
                msg->camera_pose.position.y,
                msg->camera_pose.position.z
            );

            Eigen::Quaterniond q(
                msg->camera_pose.orientation.w,
                msg->camera_pose.orientation.x,
                msg->camera_pose.orientation.y,
                msg->camera_pose.orientation.z
            );
            Eigen::Vector3d look_dir = q * Eigen::Vector3d(1, 0, 0);

            // 2. Feed the Carver State
            carver_.addCamCenter(cam_center); 
            int current_cam_idx = carver_.numCams() - 1;

            auto current_rays = carver_.getPrincipleRays();
            current_rays.push_back(look_dir);
            carver_.setPrincipleRays(current_rays);

            // 3. Process Incoming Points with 3D Voxel Deduplication
            double voxel_size = 0.05; // 5cm resolution
            
            for (size_t i = 0; i < msg->points.size(); ++i) {
                if (std::isnan(msg->points[i].x) || std::isnan(msg->points[i].y) || std::isnan(msg->points[i].z)) {
                    continue;
                }
                if (msg->points[i].x == 0.0 && msg->points[i].y == 0.0 && msg->points[i].z == 0.0) {
                    continue;
                }
                
                // Calculate Voxel Key
                VoxelKey v_key = {
                    static_cast<int>(std::floor(msg->points[i].x / voxel_size)),
                    static_cast<int>(std::floor(msg->points[i].y / voxel_size)),
                    static_cast<int>(std::floor(msg->points[i].z / voxel_size))
                };

                int local_idx;

                if (voxel_map_.count(v_key) > 0) {
                    // Reuse existing vertex for this voxel
                    local_idx = voxel_map_[v_key];
                    obs_count_[local_idx]++;
                    last_seen_kf_[local_idx] = keyframe_count_;
                } else {
                    // Create a new vertex at the actual point location (no more grid snapping!)
                    double px = msg->points[i].x;
                    double py = msg->points[i].y;
                    double pz = msg->points[i].z;
                    
                    // Slight perturbation to avoid exact duplicate vertices in Delaunay
                    px += ((rand() % 1000) - 500) * 1e-7;
                    py += ((rand() % 1000) - 500) * 1e-7;
                    pz += ((rand() % 1000) - 500) * 1e-7;
                    
                    carver_.addPoint(Eigen::Vector3d(px, py, pz));
                    local_idx = carver_.numPoints() - 1;
                    voxel_map_[v_key] = local_idx; 
                    
                    obs_count_.push_back(1);
                    last_seen_kf_.push_back(keyframe_count_);
                }
                
                // Every camera observation still contributes its line-of-sight
                carver_.addVisibilityPair(current_cam_idx, local_idx);
            }

            // 4. Update the Delaunay triangulation incrementally
            carver_.IterateTetrahedronMethod(dt_, vecVertexHandles_, current_cam_idx);

            if (keyframe_count_ % process_every_n_frames_ != 0) {
                return; 
            }

            // 5. Extract Isosurface
            std::list<Eigen::Vector3d> tris;
            std::vector<Eigen::Vector3d> points_copy = carver_.getPoints();
            carver_.tetsToTris(dt_, points_copy, tris, 1);
            
            // 6. Near-Field Clipper
            double near_clip_dist = 0.10; 
            std::list<Eigen::Vector3d> filtered_tris;
            const auto& cams = carver_.getCamCenters(); 
            
            for (const auto& tri : tris) {
                int i0 = std::round(tri.x());
                int i1 = std::round(tri.y());
                int i2 = std::round(tri.z());
                
                if (i0 < 0 || i1 < 0 || i2 < 0 || (size_t)i0 >= points_copy.size() || (size_t)i1 >= points_copy.size() || (size_t)i2 >= points_copy.size()) continue;

                Eigen::Vector3d centroid = (points_copy[i0] + points_copy[i1] + points_copy[i2]) / 3.0;
                
                double min_cam_dist = std::numeric_limits<double>::max();
                for (const auto& cam : cams) {
                    double dist = (centroid - cam).norm();
                    if (dist < min_cam_dist) min_cam_dist = dist;
                }
                
                if (min_cam_dist > near_clip_dist) {
                    filtered_tris.push_back(tri);
                }
            }
            tris = filtered_tris; 

            // 7. Publish
            publish_mesh(tris);
            publish_points(msg);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Runtime Exception: %s", e.what());
        }
    }

    void publish_mesh(const std::list<Eigen::Vector3d>& tris) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "carved_mesh";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.scale.x = 1.0; marker.scale.y = 1.0; marker.scale.z = 1.0;
        marker.color.r = 0.3; marker.color.g = 0.8; marker.color.b = 0.9;
        marker.color.a = 0.6; 

        int max_idx = carver_.numPoints();
        for (const auto& tri : tris) {
            int i0 = std::round(tri.x());
            int i1 = std::round(tri.y());
            int i2 = std::round(tri.z());

            if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= max_idx || i1 >= max_idx || i2 >= max_idx) {
                continue;
            }

            Eigen::Vector3d vec0 = carver_.getPoint(i0);
            Eigen::Vector3d vec1 = carver_.getPoint(i1);
            Eigen::Vector3d vec2 = carver_.getPoint(i2);

            geometry_msgs::msg::Point p0, p1, p2;
            p0.x = vec0.x(); p0.y = vec0.y(); p0.z = vec0.z();
            p1.x = vec1.x(); p1.y = vec1.y(); p1.z = vec1.z();
            p2.x = vec2.x(); p2.y = vec2.y(); p2.z = vec2.z();

            marker.points.push_back(p0);
            marker.points.push_back(p1);
            marker.points.push_back(p2);
        }
        pub_mesh_->publish(marker);
    }

    void publish_points(const quest3carv_interfaces::msg::KeyframeData::SharedPtr& msg) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "carved_points";
        marker.id = 1; 
        
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.scale.x = 0.02; marker.scale.y = 0.02; marker.scale.z = 0.02;
        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0; 

        for (const auto& pt : msg->points) {
            marker.points.push_back(pt);
        }
        pub_points_->publish(marker);
    }

    rclcpp::Subscription<quest3carv_interfaces::msg::KeyframeData>::SharedPtr sub_kf_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mesh_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_points_;
    
    dlovi::FreespaceDelaunayAlgorithm carver_;
    dlovi::FreespaceDelaunayAlgorithm::Delaunay3 dt_;
    std::vector<dlovi::FreespaceDelaunayAlgorithm::Delaunay3::Vertex_handle> vecVertexHandles_;

    // Voxel map for 3D deduplication
    std::unordered_map<VoxelKey, int, VoxelKeyHash> voxel_map_; 
    
    std::vector<int> obs_count_;
    std::vector<int> last_seen_kf_;
    
    int keyframe_count_;
    int process_every_n_frames_; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarvingNode>());
    rclcpp::shutdown();
    return 0;
}