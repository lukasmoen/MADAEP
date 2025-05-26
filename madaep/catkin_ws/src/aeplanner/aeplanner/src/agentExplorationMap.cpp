#include <aeplanner/agentExplorationMap.h>
#include <algorithm>
#include <cmath>

namespace aeplanner {

AgentExplorationMap::AgentExplorationMap(ros::NodeHandle& nh) : nh_(nh) {
    // Load parameters
    nh_.param<double>("gaussian_map/resolution", resolution_, 0.5);
    nh_.param<double>("gaussian_map/threshold", threshold_, 0.3);
    nh_.param<double>("gaussian_map/max_gaussians", max_gaussians_, 100.0); 
    nh_.param<double>("gaussian_map/sigma_x", default_sigma_x_, 2.0);
    nh_.param<double>("gaussian_map/sigma_y", default_sigma_y_, 2.0);
    nh_.param<double>("gaussian_map/sigma_z", default_sigma_z_, 1.0);
    nh_.param<double>("gaussian_map/default_weight", default_weight_, 1.0);
    
    // Initialize map bounds from SDF map parameters if available
    double x_min, y_min, z_min, x_max, y_max, z_max;
    if (nh_.hasParam("/exploration_node_1/sdf_map/map_size_x")) {
        nh_.getParam("/exploration_node_1/sdf_map/map_size_x", x_max);
        nh_.getParam("/exploration_node_1/sdf_map/map_size_y", y_max);
        nh_.getParam("/exploration_node_1/sdf_map/map_size_z", z_max);
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
    
    // Initialize grid
    initializeGrid();
    
    // Setup visualization
    gaussian_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("agent_exploration_gaussians", 1);
    
    ROS_INFO("[AgentExplorationMap] Initialized with threshold: %.2f, max_gaussians: %d", 
             threshold_, (int)max_gaussians_);
}

AgentExplorationMap::~AgentExplorationMap() {
    // Clean up resources if needed
}

void AgentExplorationMap::initializeGrid() {
    // Calculate grid dimensions based on bounds and resolution
    grid_size_x_ = ceil((map_max_.x() - map_min_.x()) / resolution_);
    grid_size_y_ = ceil((map_max_.y() - map_min_.y()) / resolution_);
    grid_size_z_ = ceil((map_max_.z() - map_min_.z()) / resolution_);
    
    // Initialize grid with zeros
    size_t total_cells = grid_size_x_ * grid_size_y_ * grid_size_z_;
    grid_.resize(total_cells);
    
    // Initialize all cells to zero
    for (auto& cell : grid_) {
        cell = 0.0;
    }
    
    ROS_INFO("[AgentExplorationMap] Grid initialized with dimensions: %d x %d x %d (%zu cells)",
             grid_size_x_, grid_size_y_, grid_size_z_, total_cells);
}

bool AgentExplorationMap::worldToGrid(const Eigen::Vector3d& world_pos, int& x, int& y, int& z) const {
    // Convert world coordinates to grid indices
    x = static_cast<int>((world_pos.x() - map_min_.x()) / resolution_);
    y = static_cast<int>((world_pos.y() - map_min_.y()) / resolution_);
    z = static_cast<int>((world_pos.z() - map_min_.z()) / resolution_);
    
    // Check if within grid bounds
    return isValidGridIndex(x, y, z);
}

void AgentExplorationMap::gridToWorld(int x, int y, int z, Eigen::Vector3d& world_pos) const {
    // Convert grid indices to world coordinates (cell center)
    world_pos.x() = map_min_.x() + (x + 0.5) * resolution_;
    world_pos.y() = map_min_.y() + (y + 0.5) * resolution_;
    world_pos.z() = map_min_.z() + (z + 0.5) * resolution_;
}

bool AgentExplorationMap::isValidGridIndex(int x, int y, int z) const {
    return (x >= 0 && x < grid_size_x_ &&
            y >= 0 && y < grid_size_y_ &&
            z >= 0 && z < grid_size_z_);
}

int AgentExplorationMap::gridIndex(int x, int y, int z) const {
    return x + y * grid_size_x_ + z * grid_size_x_ * grid_size_y_;
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
    blob.index = current_index_++;  // Assign a unique index for ordering
    
    // Add to Gaussians list (for visualization)
    gaussians_.push_back(blob);
    
    // Maintain a maximum number of Gaussians by removing oldest ones
    if (gaussians_.size() > max_gaussians_) {
        removeOldestGaussians();
    }
    
    // Clear the grid and rebuild it from current Gaussians
    refreshGrid();
    
    // Publish visualization
    publishGaussianMarkers();
    
    ROS_DEBUG_STREAM("[AgentExplorationMap] Added Gaussian at position: ["
                   << position.transpose() << "], weight: " << blob.weight);
}

void AgentExplorationMap::refreshGrid() {
    // Clear the grid
    std::fill(grid_.begin(), grid_.end(), 0.0);
    
    // Apply all current Gaussians to the grid
    for (const auto& gaussian : gaussians_) {
        updateGridWithGaussian(gaussian);
    }
}

void AgentExplorationMap::updateGridWithGaussian(const GaussianBlob& gaussian) {
    // Ensure covariance is usable
    Eigen::Matrix3d cov = gaussian.covariance;
    if (cov.determinant() < 1e-6) {
        cov.diagonal().array() += 1e-6;
    }
    
    // Calculate the bounds of grid cells that will be affected by this Gaussian
    // (using 3-sigma rule for a cutoff)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
    Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
    double max_sigma = std::sqrt(eigenvalues.maxCoeff()) * 3.0;
    
    // Define the bounding box (in grid coordinates)
    int min_x, min_y, min_z, max_x, max_y, max_z;
    worldToGrid(gaussian.mean - Eigen::Vector3d(max_sigma, max_sigma, max_sigma), min_x, min_y, min_z);
    worldToGrid(gaussian.mean + Eigen::Vector3d(max_sigma, max_sigma, max_sigma), max_x, max_y, max_z);
    
    // Clamp to grid bounds
    min_x = std::max(0, min_x);
    min_y = std::max(0, min_y);
    min_z = std::max(0, min_z);
    max_x = std::min(grid_size_x_ - 1, max_x);
    max_y = std::min(grid_size_y_ - 1, max_y);
    max_z = std::min(grid_size_z_ - 1, max_z);
    
    // Inverse of covariance for Mahalanobis distance
    Eigen::Matrix3d cov_inv = cov.inverse();
    
    // Update each cell in the bounding box
    for (int z = min_z; z <= max_z; z++) {
        for (int y = min_y; y <= max_y; y++) {
            for (int x = min_x; x <= max_x; x++) {
                // Get world coordinates for this cell
                Eigen::Vector3d world_pos;
                gridToWorld(x, y, z, world_pos);
                
                // Compute Mahalanobis distance
                Eigen::Vector3d diff = world_pos - gaussian.mean;
                double mahalanobis_squared = diff.transpose() * cov_inv * diff;
                
                // Skip if too far (more than 3-sigma)
                if (mahalanobis_squared > 9.0) continue;
                
                // Compute Gaussian value
                double gaussian_value = gaussian.weight * exp(-0.5 * mahalanobis_squared);
                
                // Add to existing cell value
                int idx = gridIndex(x, y, z);
                grid_[idx] += gaussian_value;
            }
        }
    }
}

void AgentExplorationMap::addGoalsAsGaussians(const std::vector<aeplanner::Goals>& goals, const int id, const std::string& agent_ns) {
    // Check if this goal has already been added
    if (id_map.find(agent_ns) != id_map.end() && id_map[agent_ns] == id) {
        return;
    }
    
    for (const auto& goal_msg : goals) {
        for (const auto& g : goal_msg.goals) {
            Eigen::Vector3d position(g.x, g.y, g.z);
            addGaussian(position);
        }
    }
    
    id_map[agent_ns] = id;
}

double AgentExplorationMap::queryGaussianValue(const Eigen::Vector3d& position) const {
    int x, y, z;
    
    // Convert world position to grid indices
    if (!worldToGrid(position, x, y, z)) {
        return 0.0; // Outside grid bounds
    }
    
    // Get the cell value
    return grid_[gridIndex(x, y, z)];
}

bool AgentExplorationMap::isBeingExplored(const Eigen::Vector3d& position) const {
    return queryGaussianValue(position) > threshold_;
}

double AgentExplorationMap::adjustInformationGain(const Eigen::Vector3d& position, double original_gain) const {
    double gaussian_value = queryGaussianValue(position);
    
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

void AgentExplorationMap::removeOldestGaussians() {
    // Sort by index (which reflects the order they were added)
    std::sort(gaussians_.begin(), gaussians_.end(), 
              [](const GaussianBlob& a, const GaussianBlob& b) {
                  return a.index < b.index;
              });
    
    // Remove the oldest Gaussians until we're below the maximum
    if (gaussians_.size() > max_gaussians_) {
        size_t to_remove = gaussians_.size() - max_gaussians_;
        gaussians_.erase(gaussians_.begin(), gaussians_.begin() + to_remove);
        
        ROS_DEBUG("[AgentExplorationMap] Removed %zu oldest Gaussians", to_remove);
    }
}

void AgentExplorationMap::clearAllGaussians() {
    gaussians_.clear();
    std::fill(grid_.begin(), grid_.end(), 0.0);
    
    // Publish empty visualization
    publishGaussianMarkers();
    
    ROS_INFO("[AgentExplorationMap] Cleared all Gaussians");
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
    for (const auto& gaussian : gaussians_) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
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
        
        // Color and transparency based on weight 
        double alpha = std::min(0.7, gaussian.weight);
        
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