#include <aeplanner/agentExplorationMap.h>
#include <algorithm>
#include <cmath>

namespace aeplanner
{

AgentExplorationMap::AgentExplorationMap(ros::NodeHandle& nh) : nh_(nh) {
    // Load parameters
    nh_.param<double>("gaussian_map/resolution", resolution_, 0.5);
    nh_.param<double>("gaussian_map/threshold", threshold_, 0.3);
    nh_.param<double>("gaussian_map/time_decay", time_decay_, 60.0); // seconds
    nh_.param<double>("gaussian_map/sigma_x", default_sigma_x_, 2.0);
    nh_.param<double>("gaussian_map/sigma_y", default_sigma_y_, 2.0);
    nh_.param<double>("gaussian_map/sigma_z", default_sigma_z_, 1.0);
    nh_.param<double>("gaussian_map/default_weight", default_weight_, 1.0);
    
    // Initialize map bounds from SDF map parameters if available
    double x_min, y_min, z_min, x_max, y_max, z_max;
    if (nh_.hasParam("/sdf_map/map_size_x")) {
        nh_.getParam("/sdf_map/map_size_x", x_max);
        nh_.getParam("/sdf_map/map_size_y", y_max);
        nh_.getParam("/sdf_map/map_size_z", z_max);
        x_min = -x_max;
        y_min = -y_max;
        z_min = 0.0; // Assuming ground is at z=0
        
        map_min_ = Eigen::Vector3d(x_min, y_min, z_min);
        map_max_ = Eigen::Vector3d(x_max, y_max, z_max);
        
        ROS_INFO_STREAM("[AgentExplorationMap] Initialized with map bounds: ["
                      << map_min_.transpose() << "] to ["
                      << map_max_.transpose() << "]");
    } else {
        // Default map size if not specified
        map_min_ = Eigen::Vector3d(-50, -50, 0);
        map_max_ = Eigen::Vector3d(50, 50, 10);
        ROS_WARN("[AgentExplorationMap] No SDF map parameters found, using default bounds");
    }
    
    // Setup visualization
    gaussian_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("agent_exploration_gaussians", 1);
    
    ROS_INFO("[AgentExplorationMap] Initialized with threshold: %.2f, time_decay: %.2f seconds", 
            threshold_, time_decay_);

    std::map<std::string, int> id_map;
}

AgentExplorationMap::~AgentExplorationMap() {
    // Clean up resources if needed
}

void AgentExplorationMap::addGaussian(const Eigen::Vector3d& position, 
                                     const Eigen::Matrix3d& covariance,
                                     double weight) {
    GaussianBlob blob;
    blob.mean = position;
    
    // If no custom covariance provided, use default
    if (covariance.isZero()) {
        blob.covariance = Eigen::Matrix3d::Zero();
        blob.covariance(0, 0) = default_sigma_x_ * default_sigma_x_;
        blob.covariance(1, 1) = default_sigma_y_ * default_sigma_y_;
        blob.covariance(2, 2) = default_sigma_z_ * default_sigma_z_;
    } else {
        blob.covariance = covariance;
    }
    
    blob.weight = (weight < 0) ? default_weight_ : weight;
    blob.timestamp = ros::Time::now();
    
    gaussians_.push_back(blob);
    
    // Clean up old Gaussians
    removeOldGaussians();
    
    // Publish visualization
    publishGaussianMarkers();
    
    ROS_DEBUG_STREAM("[AgentExplorationMap] Added Gaussian at position: ["
                   << position.transpose() << "], weight: " << blob.weight);
}

void AgentExplorationMap::addGoalsAsGaussians(const std::vector<aeplanner::Goals>& goals, const int id, const std::string& agent_ns) {

    if (id_map[agent_ns] == id){
        // goal already added
        return;
    }
    for (const auto& goal_msg : goals) {
        for (const auto& g : goal_msg.goals) {  // ✅ Assuming goal_msg.goals is a std::vector<Goal>
            Eigen::Vector3d position(g.x, g.y, g.z);
            addGaussian(position);
        }
    }
    id_map[agent_ns] = id;
  }

double AgentExplorationMap::queryGaussianValue(const Eigen::Vector3d& position) const {
    double value = 0.0;
    ros::Time now = ros::Time::now();
    
    for (const auto& gaussian : gaussians_) {
        // Apply time decay
        double time_factor = 1.0;
        if (time_decay_ > 0) {
            double dt = (now - gaussian.timestamp).toSec();
            time_factor = exp(-dt / time_decay_);
        }
        
        // Skip if the time factor is negligible
        if (time_factor < 0.01) continue;
        
        // Compute Mahalanobis distance
        Eigen::Vector3d diff = position - gaussian.mean;
        
        // Ensure covariance is invertible (add small epsilon if needed)
        Eigen::Matrix3d cov = gaussian.covariance;
        if (cov.determinant() < 1e-6) {
            cov.diagonal().array() += 1e-6;
        }
        
        double mahalanobis_squared = diff.transpose() * cov.inverse() * diff;
        
        // Compute Gaussian value with time decay
        double gaussian_value = gaussian.weight * time_factor * 
                               exp(-0.5 * mahalanobis_squared);
        
        // Sum up values (can be modified for different combination strategies)
        value += gaussian_value;
    }
    
    return value;
}

bool AgentExplorationMap::isBeingExplored(const Eigen::Vector3d& position) const {
    return queryGaussianValue(position) > threshold_;
}

double AgentExplorationMap::adjustInformationGain(const Eigen::Vector3d& position, double original_gain) const {
    double gaussian_value = queryGaussianValue(position);
    // ROS_INFO_STREAM("Gaussian value: "<< gaussian_value);
    if (gaussian_value > threshold_) {
        return 0.0; // Zero gain if above threshold
    } else if (gaussian_value > 0.0) {
        // Linear scaling of gain based on gaussian value
        return original_gain * (1.0 - gaussian_value / threshold_);
    } else {
        // No adjustment needed if no overlap
        return original_gain;
    }
}

void AgentExplorationMap::removeOldGaussians() {
    size_t before = gaussians_.size();
    ros::Time cutoff = ros::Time::now() - ros::Duration(time_decay_ * 3); // 3x decay time
    
    gaussians_.erase(
        std::remove_if(gaussians_.begin(), gaussians_.end(),
            [&cutoff](const GaussianBlob& blob) {
                return blob.timestamp < cutoff;
            }),
        gaussians_.end()
    );
    
    size_t after = gaussians_.size();
    if (before != after) {
        ROS_DEBUG("[AgentExplorationMap] Removed %zu old Gaussians", before - after);
    }
}

void AgentExplorationMap::publishGaussianMarkers() {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    
    // First add a delete marker to clear previous markers
    if (!gaussians_.empty()) {
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "world";
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.ns = "agent_exploration_gaussians";
        delete_marker.id = 0;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);
    }
    
    // Now add markers for each Gaussian
    ros::Time now = ros::Time::now();
    
    for (const auto& gaussian : gaussians_) {
        // Calculate age factor
        double age = (now - gaussian.timestamp).toSec();
        double age_factor = exp(-age / time_decay_);
        
        // Skip if too faded
        if (age_factor < 0.05) continue;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = now;
        marker.ns = "agent_exploration_gaussians";
        marker.id = ++id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Position
        marker.pose.position.x = gaussian.mean(0);
        marker.pose.position.y = gaussian.mean(1);
        marker.pose.position.z = gaussian.mean(2);
        marker.pose.orientation.w = 1.0;
        
        // Size based on covariance - use eigenvalues for visualization
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(gaussian.covariance);
        
        // Use 2-sigma for visualization (covers ~95% of probability mass)
        double scale_factor = 2.0;
        
        if (eigensolver.info() == Eigen::Success) {
            // Get eigenvalues and vectors
            Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
            
            // Scale is 2*sqrt(eigenvalue) for each axis (2-sigma ellipsoid)
            marker.scale.x = scale_factor * std::sqrt(std::max(eigenvalues(0), 0.01));
            marker.scale.y = scale_factor * std::sqrt(std::max(eigenvalues(1), 0.01));
            marker.scale.z = scale_factor * std::sqrt(std::max(eigenvalues(2), 0.01));
            
            // TODO: For proper ellipsoid visualization, we would need to set the orientation
            // based on eigenvectors, but this is complex to visualize in RViz
        } else {
            // Fallback if eigendecomposition fails
            marker.scale.x = scale_factor * default_sigma_x_;
            marker.scale.y = scale_factor * default_sigma_y_;
            marker.scale.z = scale_factor * default_sigma_z_;
        }
        
        // Color and transparency based on weight and time decay
        double alpha = std::min(0.7, gaussian.weight * age_factor);
        
        marker.color.r = 0.0;
        marker.color.g = 0.8;
        marker.color.b = 0.8;
        marker.color.a = alpha;
        
        marker.lifetime = ros::Duration(2.0);
        
        marker_array.markers.push_back(marker);
    }
    
    // Only publish if we have markers
    if (!marker_array.markers.empty()) {
        gaussian_marker_pub_.publish(marker_array);
    }
}

} // namespace aeplanner