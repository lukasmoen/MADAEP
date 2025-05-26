#include <ros/ros.h>
#include <aeplanner_evaluation/Coverage.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>
#include <plan_env/multi_map_manager.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

void octomapCallback(const octomap_msgs::Octomap& msg);
void sdfMapCallback(const sensor_msgs::PointCloud2& msg);
fast_planner::SDFMap* sdf_map_ = nullptr;

class CoverageEvaluator{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_, sdfmap_sub_,odom_sub_;
    ros::ServiceServer coverage_srv_;

    std::vector<double> min_vals;
    std::vector<double> max_vals;

    Eigen::Vector4d current_state_;

    double volume_;
    double coverage_m3_, coverage_p_, free_, occupied_, unmapped_;


  public:
    CoverageEvaluator(ros::NodeHandle& nh) : nh_(nh),
                                             octomap_sub_(nh_.subscribe("octomap", 10, &CoverageEvaluator::octomapCallback, this)),
                                             sdfmap_sub_(nh_.subscribe("occupancy_all", 10, &CoverageEvaluator::sdfMapCallback, this)),
                                             odom_sub_(nh_.subscribe("odom", 10, &CoverageEvaluator::agentPoseCallback, this)),
                                             coverage_srv_(nh.advertiseService("get_coverage", &CoverageEvaluator::coverageSrvCallback, this)){
      
      
      if (!nh.getParam("boundary/volume", volume_)) {
        ROS_ERROR("No volume found...");
      }
      if (!nh.getParam("boundary/min", min_vals)) {
        ROS_ERROR("No minimum boundary values found...");
      }
      if (!nh.getParam("boundary/max", max_vals)) {
        ROS_ERROR("No maximum boundary values found...");
      }

   
    }

    void octomapCallback(const octomap_msgs::Octomap& msg);
    void sdfMapCallback(const sensor_msgs::PointCloud2& msg);
    void agentPoseCallback(const nav_msgs::Odometry& msg);
    bool coverageSrvCallback(aeplanner_evaluation::Coverage::Request& request, aeplanner_evaluation::Coverage::Response& response);
  
};

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "coverage_evaluation");
  ROS_INFO("coverage_evaluation initialized");
  ros::NodeHandle nh;

  CoverageEvaluator ce_(nh);
       
  ROS_INFO("[EVAL] Initializing SDF Map...");
  try {
    sdf_map_ = new fast_planner::SDFMap(); 
    sdf_map_->initMap(nh);
  
    ROS_INFO("[EVAL] SDF Map Initialized Successfully.");
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to initialize SDF Map: %s", e.what());
  }

  ros::spin();
  
}

void CoverageEvaluator::octomapCallback(const octomap_msgs::Octomap& msg){
  return;
  std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree> (dynamic_cast<octomap::OcTree*> (octomap_msgs::msgToMap(msg)));
  double occupied_space = 0.0;
  double free_space = 0.0;

  // Map limits
  octomap::point3d min(min_vals[0], min_vals[1], min_vals[2]);
  octomap::point3d max(max_vals[0], max_vals[1], max_vals[2]);

    for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min, max), end = octree->end_leafs_bbx(); it != end; ++it) {
      const octomap::point3d& pos = it.getCoordinate(); // get the position of the leaf node
      // Check if position is within the bounding box
      if (pos.x() > min.x() && pos.y() > min.y() && pos.z() > min.z() && pos.x() < max.x() && pos.y() < max.y() && pos.z() < max.z()) {
        double side_length = it.getSize();
        if (octree->isNodeOccupied(*it)){ // occupied leaf node
          occupied_space += pow(side_length,3);
        } else { // free leaf node
          free_space += pow(side_length,3);
        }
      }
    }

    double mapped_space = occupied_space + free_space;
    double unmapped_space = volume_ - mapped_space;
    double coverage = 100.0 * mapped_space / volume_;

    // Set global variables
    coverage_m3_ = mapped_space;
    coverage_p_ = coverage;
    free_ = free_space;
    occupied_ = occupied_space;
    unmapped_ = unmapped_space;
}

void CoverageEvaluator::sdfMapCallback(const sensor_msgs::PointCloud2& msg) {
  if (!sdf_map_) {
    ROS_ERROR("SDF Map is not initialized!");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(msg, pcl_cloud);

  if (pcl_cloud.empty()) {
    ROS_WARN("[EVAL] Received empty point cloud in sdfMapCallback!");
    return;
  }

  int point_num = pcl_cloud.size();
  Eigen::Vector3d origin(current_state_[0], current_state_[1], current_state_[2]);

  // Mark occupied points in the SDF
  for (const auto& pt : pcl_cloud.points) {
    Eigen::Vector3d obs_pt(pt.x, pt.y, pt.z);
    sdf_map_->setOccupied(obs_pt);
  }

  // Process the point cloud to update the SDF map
  sdf_map_->inputPointCloud(pcl_cloud, point_num, origin);

  double occupied_space = 0.0;
  double free_space = 0.0;

  double resolution = sdf_map_->getResolution();
  Eigen::Vector3d min(min_vals[0], min_vals[1], min_vals[2]);
  Eigen::Vector3d max(max_vals[0], max_vals[1], max_vals[2]);

  // Iterate through the defined bounding box
  // for (double x = min.x(); x <= max.x(); x += resolution) {
  //   for (double y = min.y(); y <= max.y(); y += resolution) {
  //     for (double z = min.z(); z <= max.z(); z += resolution) {
  //       Eigen::Vector3d point(x, y, z);
        
  //       if (!sdf_map_->isInMap(point)) continue;

  //       int occupancy = sdf_map_->getOccupancy(point);
  //       double voxel_volume = std::pow(resolution, 3);

  //       if (occupancy == 2) { // Occupied
  //         occupied_space += voxel_volume;
  //       } else if (occupancy == 1) { // Free
  //         free_space += voxel_volume;
  //       }
  //     }
  //   }
  // }
  double voxel_volume = std::pow(resolution, 3);
  size_t total_voxels = sdf_map_->md_->occupancy_buffer_.size();
  
  for (size_t idx = 0; idx < total_voxels; ++idx) {

    // from sdf_map.h
    double occ = sdf_map_->md_->occupancy_buffer_[idx];
    if (occ < sdf_map_->mp_->clamp_min_log_ - 1e-3) 
    {continue;}
    else if (occ > sdf_map_->mp_->min_occupancy_log_) 
    {occupied_space += voxel_volume;}
    else {free_space += voxel_volume;}

  }
  double mapped_space = occupied_space + free_space;
  double unmapped_space = volume_ - mapped_space;
  double coverage = 100.0 * mapped_space / volume_;

  // ROS_INFO_STREAM("Mapped space "<< mapped_space);
  // ROS_INFO_STREAM("Unmapped space "<< unmapped_space);
  // ROS_INFO_STREAM("Coverage "<< coverage);


  // Set global variables
  coverage_m3_ = mapped_space;
  coverage_p_ = coverage;
  free_ = free_space;
  occupied_ = occupied_space;
  unmapped_ = unmapped_space;
}


void CoverageEvaluator::agentPoseCallback(const nav_msgs::Odometry& msg)
{
  current_state_[0] = msg.pose.pose.position.x;
  current_state_[1] = msg.pose.pose.position.y;
  current_state_[2] = msg.pose.pose.position.z;
  current_state_[3] = tf2::getYaw(msg.pose.pose.orientation);
}

bool CoverageEvaluator::coverageSrvCallback(aeplanner_evaluation::Coverage::Request& request, aeplanner_evaluation::Coverage::Response& response){
  
  response.coverage_m3 = coverage_m3_;
  response.coverage_p = coverage_p_;
  response.free     = free_;
  response.occupied = occupied_;
  response.unmapped = unmapped_;
  return true;
}
