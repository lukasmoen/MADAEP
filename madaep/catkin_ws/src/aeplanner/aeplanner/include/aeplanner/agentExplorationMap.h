#ifndef AGENT_EXPLORATION_MAP_H
#define AGENT_EXPLORATION_MAP_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <string>
#include <visualization_msgs/MarkerArray.h>
#include <aeplanner/Goals.h>  // Assuming this is your custom message type

namespace aeplanner {

class AgentExplorationMap {
public:
    AgentExplorationMap(ros::NodeHandle& nh);
    ~AgentExplorationMap();

    /**
     * Add a Gaussian blob at the specified position
     * @param position Center of the Gaussian in world coordinates
     * @param covariance Optional covariance matrix (if zero, defaults will be used)
     * @param weight Weight of the Gaussian (if negative, default will be used)
     */
    void addGaussian(const Eigen::Vector3d& position, 
                     const Eigen::Matrix3d& covariance = Eigen::Matrix3d::Zero(),
                     double weight = -1.0);

    /**
     * Add exploration goals as Gaussians
     * @param goals Vector of goal messages
     * @param id Goal ID
     * @param agent_ns Agent namespace
     */
    void addGoalsAsGaussians(const std::vector<aeplanner::Goals>& goals, const int id, const std::string& agent_ns);

    /**
     * Check if a position is being explored by other agents
     * @param position Position to check in world coordinates
     * @return true if position is being explored (above threshold)
     */
    bool isBeingExplored(const Eigen::Vector3d& position) const;

    /**
     * Adjust information gain based on exploration status
     * @param position Position to check in world coordinates
     * @param original_gain Original information gain
     * @return Adjusted gain value (reduced or zero if being explored)
     */
    double adjustInformationGain(const Eigen::Vector3d& position, double original_gain) const;

    /**
     * Query the Gaussian value at a specified position
     * @param position Position to query in world coordinates
     * @return Gaussian value at the position
     */
    double queryGaussianValue(const Eigen::Vector3d& position) const;

    /**
     * Clear all Gaussians from the map
     */
    void clearAllGaussians();

private:
    struct GaussianBlob {
        Eigen::Vector3d mean;
        Eigen::Matrix3d covariance;
        double weight;
        size_t index;  // For keeping track of order
    };

    // Initialize the grid
    void initializeGrid();

    // Refresh the entire grid based on current Gaussians
    void refreshGrid();

    // Update grid with a Gaussian blob
    void updateGridWithGaussian(const GaussianBlob& gaussian);

    // Remove oldest Gaussians to stay under max limit
    void removeOldestGaussians();

    // Publish visualization markers
    void publishGaussianMarkers();

    // Convert between world and grid coordinates
    bool worldToGrid(const Eigen::Vector3d& world_pos, int& x, int& y, int& z) const;
    void gridToWorld(int x, int y, int z, Eigen::Vector3d& world_pos) const;
    
    // Grid utility functions
    bool isValidGridIndex(int x, int y, int z) const;
    int gridIndex(int x, int y, int z) const;

    // Node handle
    ros::NodeHandle& nh_;

    // Publishers
    ros::Publisher gaussian_marker_pub_;

    // Parameters
    double resolution_;
    double threshold_;
    double max_gaussians_;  
    double default_sigma_x_;
    double default_sigma_y_;
    double default_sigma_z_;
    double default_weight_;

    // Map bounds
    Eigen::Vector3d map_min_;
    Eigen::Vector3d map_max_;

    // Grid dimensions
    int grid_size_x_;
    int grid_size_y_;
    int grid_size_z_;

    // Grid storage - simplified to just double values
    std::vector<double> grid_;

    // Store Gaussians for visualization
    std::vector<GaussianBlob> gaussians_;

    // Track IDs to avoid duplicate processing
    std::unordered_map<std::string, int> id_map;
    
    // Counter for assigning unique indices to Gaussians
    size_t current_index_ = 0;
};

} // namespace aeplanner

#endif // AGENT_EXPLORATION_MAP_H