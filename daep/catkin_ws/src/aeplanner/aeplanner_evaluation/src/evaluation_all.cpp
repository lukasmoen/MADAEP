#include <ros/ros.h>
#include <aeplanner_evaluation/Coverage.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/sdf_map.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <std_msgs/String.h>
#include <sstream>
#include <chrono>

class CoverageEvaluator {
private:
  ros::NodeHandle nh_;
  ros::ServiceServer coverage_srv_;

  std::vector<ros::Subscriber> pc_subs_;
  std::vector<ros::Subscriber> odom_subs_;
  ros::Publisher completed_pub, map_all_pub_;
  ros::Timer stationary_check_timer_,stationary_check_timer_2;

  std::unordered_map<std::string, Eigen::Vector3d> drone_odoms_;
  std::unordered_map<std::string, Eigen::Vector3d> drone_prev_positions_;
  std::unordered_map<std::string, ros::Time> drone_stationary_since_;
  double visualization_truncate_height_, visualization_truncate_low_;
  // Parameters for stationary detection
  double position_threshold_;  // Position difference threshold to consider drone stationary (meters)
  double stationary_time_threshold_;  // Time threshold for all drones to be stationary (seconds)
  bool completion_message_sent_;  // Flag to track if completion message has been sent

  std::shared_ptr<fast_planner::SDFMap> sdf_map_;

  double volume_;
  std::vector<double> min_vals_, max_vals_;
  int drone_num_;

  double coverage_m3_, coverage_p_, free_, occupied_, unmapped_;

public:
  CoverageEvaluator(ros::NodeHandle& nh) : nh_(nh), completion_message_sent_(false) {
    if (!nh_.getParam("boundary/volume", volume_) ||
        !nh_.getParam("boundary/min", min_vals_) ||
        !nh_.getParam("boundary/max", max_vals_)) {
      ROS_ERROR("Missing boundary parameters.");
    }

    nh_.param("drone_num", drone_num_, 1);
    
    // Parameters for stationary detection
    nh_.param("position_threshold", position_threshold_, 0.1);  // Default: 10cm threshold
    nh_.param("stationary_time_threshold", stationary_time_threshold_, 300.0);  // Default: 5 minutes (300 seconds)
    
    ROS_INFO("Stationary detection parameters: position_threshold = %.2f m, time_threshold = %.2f s", 
             position_threshold_, stationary_time_threshold_);

    completed_pub = nh.advertise<std_msgs::String>("/exploration_completed", 1);
    map_all_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_", 10);
    
    // Create a timer to periodically check if all drones are stationary
    stationary_check_timer_ = nh_.createTimer(
        ros::Duration(1.0),  // Check every 1 second
        &CoverageEvaluator::checkStationaryStatus, this);

    stationary_check_timer_2 = nh_.createTimer(
      ros::Duration(1.0),  // Check every 1 second
      &CoverageEvaluator::updateCoverageStats, this);

    nh.param("/exploration_node_1/map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
    nh.param("/exploration_node_1/map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);

    sdf_map_ = std::make_shared<fast_planner::SDFMap>();
    sdf_map_->initMap(nh_);

    for (int i = 1; i <= drone_num_; ++i) {
      std::string drone_name = "drone" + std::to_string(i);
      drone_odoms_[drone_name] = Eigen::Vector3d(0, 0, 0);
      drone_prev_positions_[drone_name] = Eigen::Vector3d(0, 0, 0);
      drone_stationary_since_[drone_name] = ros::Time::now();

      std::string pc_topic = "/sdf_map/occupancy_local_" + std::to_string(i);
      std::string odom_topic = "/" + drone_name + "/odom";

      pc_subs_.emplace_back(nh_.subscribe<sensor_msgs::PointCloud2>(
          pc_topic, 10,
          [this, drone_name](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            this->pointcloudCallback(msg, drone_name);
          }));

      odom_subs_.emplace_back(nh_.subscribe<nav_msgs::Odometry>(
          odom_topic, 10,
          [this, drone_name](const nav_msgs::Odometry::ConstPtr& msg) {
            this->odomCallback(msg, drone_name);
          }));
    }

    coverage_srv_ = nh_.advertiseService("get_coverage0", &CoverageEvaluator::coverageSrvCallback, this);
  }

  void publishMapAll() {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  
    Eigen::Vector3i min_idx, max_idx;
    sdf_map_->posToIndex(sdf_map_->md_->all_min_, min_idx);
    sdf_map_->posToIndex(sdf_map_->md_->all_max_, max_idx);
  
    sdf_map_->boundIndex(min_idx);
    sdf_map_->boundIndex(max_idx);
    
  
    for (int x = min_idx[0]; x <= max_idx[0]; ++x)
      for (int y = min_idx[1]; y <= max_idx[1]; ++y)
        for (int z = min_idx[2]; z <= max_idx[2]; ++z) {
          if (sdf_map_->md_->occupancy_buffer_[sdf_map_->toAddress(x, y, z)] >
              sdf_map_->mp_->min_occupancy_log_) {
            Eigen::Vector3d pos;
            sdf_map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > visualization_truncate_height_) continue;
            if (pos(2) < visualization_truncate_low_) continue;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud1.push_back(pt);
          }
        }
    cloud1.width = cloud1.points.size();
    cloud1.height = 1;
    cloud1.is_dense = true;
    cloud1.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud1, cloud_msg);
    map_all_pub_.publish(cloud_msg);
  
  }

  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& drone_name) {
    if (!sdf_map_) {
      ROS_ERROR("SDF Map not initialized!");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    if (pcl_cloud.empty()) {
      ROS_WARN("Point cloud is empty for %s", drone_name.c_str());
      return;
    }

    Eigen::Vector3d origin = drone_odoms_[drone_name];
    for (const auto& pt : pcl_cloud.points) {
      sdf_map_->setOccupied(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }

    sdf_map_->inputPointCloud(pcl_cloud, pcl_cloud.size(), origin);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& drone_name) {
    Eigen::Vector3d current_position(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    
    // Store the current position
    drone_odoms_[drone_name] = current_position;
    
    // Check if drone has moved significantly from its previous position
    if ((drone_prev_positions_[drone_name] - current_position).norm() > position_threshold_) {
      // Drone has moved, reset the stationary timer
      drone_stationary_since_[drone_name] = ros::Time::now();
      // Update the previous position
      drone_prev_positions_[drone_name] = current_position;
    }
  }

  void checkStationaryStatus(const ros::TimerEvent&) {
    bool all_stationary = true;
    ros::Time current_time = ros::Time::now();
    ros::Duration max_stationary_time(0);
    
    // Check if all drones have been stationary for the required time
    for (int i = 1; i <= drone_num_; ++i) {
      std::string drone_name = "drone" + std::to_string(i);
      
      ros::Duration stationary_time = current_time - drone_stationary_since_[drone_name];
      
      if (stationary_time.toSec() > max_stationary_time.toSec()) {
        max_stationary_time = stationary_time;
      }
      
      if (stationary_time.toSec() < stationary_time_threshold_) {
        all_stationary = false;
        break;
      }
    }
    
    // If all drones have been stationary for the required time, publish completion message
    if (all_stationary) {
      ROS_INFO("All drones have been stationary for %.2f seconds. Publishing completion message.", max_stationary_time.toSec());
      ros::Duration(2.0).sleep();
      publishCompletionMessage();
    }
  }

  void publishCompletionMessage() {
    for (int id = 1; id <= drone_num_; ++id) {
      std_msgs::String exploration_completed_msg;
      std::stringstream ss;
      ss << "/aeplanner" << id << ",true";
      exploration_completed_msg.data = ss.str();

      completed_pub.publish(exploration_completed_msg);
      ROS_INFO("Published completion message: %s", exploration_completed_msg.data.c_str());
    }
  }

  bool coverageSrvCallback(aeplanner_evaluation::Coverage::Request& request,
                           aeplanner_evaluation::Coverage::Response& response) {
    response.coverage_m3 = coverage_m3_;
    response.coverage_p = coverage_p_;
    response.free = free_;
    response.occupied = occupied_;
    response.unmapped = unmapped_;
    return true;
  }

  void updateCoverageStats(const ros::TimerEvent&) {
    // publishMapAll();

    double occupied_space = 0.0;
    double free_space = 0.0;
  
    double resolution = sdf_map_->getResolution();
    double voxel_volume = std::pow(resolution, 3);
  
    // Lock the map during calculation to prevent concurrent modifications
    std::lock_guard<std::mutex> lock(sdf_map_->map_mutex_);
  
    // Calculate coverage only within the defined boundary
    Eigen::Vector3d boundary_min(min_vals_[0], min_vals_[1], min_vals_[2]);
    Eigen::Vector3d boundary_max(max_vals_[0], max_vals_[1], max_vals_[2]);
    
    // Convert boundary to indices
    Eigen::Vector3i idx_min, idx_max;
    sdf_map_->posToIndex(boundary_min, idx_min);
    sdf_map_->posToIndex(boundary_max, idx_max);
    
    // Ensure indices are within map bounds
    sdf_map_->boundIndex(idx_min);
    sdf_map_->boundIndex(idx_max);
    
    // Iterate through voxels only within the boundary
    for (int x = idx_min[0]; x <= idx_max[0]; ++x) {
      for (int y = idx_min[1]; y <= idx_max[1]; ++y) {
        for (int z = idx_min[2]; z <= idx_max[2]; ++z) {
          Eigen::Vector3i id(x, y, z);
          int adr = sdf_map_->toAddress(id);
          
          // Check if voxel is within map bounds
          if (!sdf_map_->isInMap(id)) continue;
          
          // Get occupancy value from buffer
          double occ = sdf_map_->md_->occupancy_buffer_[adr];
          
          // Check if voxel has been updated (not unknown)
          if (occ < sdf_map_->mp_->clamp_min_log_ - 1e-3) continue;
          
          // Classify voxel as occupied or free
          if (occ > sdf_map_->mp_->min_occupancy_log_)
            occupied_space += voxel_volume;
          else
            free_space += voxel_volume;
        }
      }
    }
  
    double mapped_space = occupied_space + free_space;
    
    // Calculate theoretical volume of the boundary
    double boundary_volume = (max_vals_[0] - min_vals_[0]) * 
                             (max_vals_[1] - min_vals_[1]) * 
                             (max_vals_[2] - min_vals_[2]);
    
    // Use the theoretical boundary volume for coverage calculation
    double unmapped_space = boundary_volume - mapped_space;
    double coverage = 100.0 * mapped_space / boundary_volume;
  
    // Update stats with the bounded values
    coverage_m3_ = mapped_space;
    coverage_p_ = coverage;
    free_ = free_space;
    occupied_ = occupied_space;
    unmapped_ = unmapped_space;

    // Check if coverage threshold is met
    // ROS_ERROR_STREAM("Coverage: " << coverage_p_);
    if (coverage_p_ >= 99) {
      publishCompletionMessage();
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "coverage_evaluation");
  ros::NodeHandle nh;

  CoverageEvaluator ce(nh);

  ros::spin();
  return 0;
}