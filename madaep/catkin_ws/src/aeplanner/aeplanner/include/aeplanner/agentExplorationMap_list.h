#ifndef AGENT_EXPLORATION_MAP_H
#define AGENT_EXPLORATION_MAP_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <aeplanner/Goals.h>

namespace aeplanner
{

/**
 * @brief Structure representing a 3D Gaussian blob used to model agent exploration areas
 */
struct GaussianBlob {
    Eigen::Vector3d mean;       // Center position of the Gaussian
    Eigen::Matrix3d covariance; // Covariance matrix defining shape/size
    double weight;              // Weight/importance of this Gaussian
    ros::Time timestamp;        // When this Gaussian was added
};

/**
 * @brief Class to manage a map of Gaussian blobs representing areas being explored by agents
 * Used to coordinate multi-agent exploration by avoiding redundant exploration
 */
class AgentExplorationMap {
public:
    /**
     * @brief Constructor initializes the exploration map
     * @param nh ROS node handle for parameter loading and publishing
     */
    AgentExplorationMap(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~AgentExplorationMap();
    
    /**
     * @brief Add a Gaussian blob at a goal position
     * @param position Center position of the Gaussian
     * @param covariance Optional covariance matrix (default: diagonal matrix with default sigma values)
     * @param weight Optional weight for this Gaussian (default: default_weight_)
     */
    void addGaussian(const Eigen::Vector3d& position, 
                     const Eigen::Matrix3d& covariance = Eigen::Matrix3d::Zero(),
                     double weight = -1.0);
    
    /**
     * @brief Add Gaussians from other agents' goals, checking for new goals via ID
     * @param goals Vector of goal messages from other agents
     */
    void addGoalsAsGaussians(const std::vector<aeplanner::Goals>& goals, const int id, const std::string& agent_ns);
    
    /**
     * @brief Query the exploration map at a position
     * @param position 3D position to query
     * @return Gaussian value at the position (higher means more likely being explored)
     */
    double queryGaussianValue(const Eigen::Vector3d& position) const;
    
    /**
     * @brief Check if a position is already being explored by another agent
     * @param position 3D position to check
     * @return true if the position is likely being explored by another agent
     */
    bool isBeingExplored(const Eigen::Vector3d& position) const;
    
    /**
     * @brief Adjust the information gain based on the Gaussian map
     * @param position 3D position to evaluate
     * @param original_gain Original information gain
     * @return Adjusted gain (zero if area is already being explored)
     */
    double adjustInformationGain(const Eigen::Vector3d& position, double original_gain) const;
    
    /**
     * @brief Remove Gaussians that are too old
     */
    void removeOldGaussians();
    
    /**
     * @brief Visualize the Gaussian blobs as RViz markers
     */
    void publishGaussianMarkers();
    
private:
    ros::NodeHandle& nh_;
    std::vector<GaussianBlob> gaussians_;
    ros::Publisher gaussian_marker_pub_;
    
    // Map bounds
    Eigen::Vector3d map_min_;
    Eigen::Vector3d map_max_;
    
    // Parameters
    double resolution_;  // Grid resolution for visualization
    double threshold_;   // Threshold above which areas are considered explored
    double time_decay_;  // Time decay parameter (seconds)
    
    // Default Gaussian parameters
    double default_sigma_x_;
    double default_sigma_y_;
    double default_sigma_z_;
    double default_weight_;
    
    // Track previously seen goals by agent ID
    std::map<std::string, std::map<int, Eigen::Vector3d>> agent_goal_ids_;
    std::map<std::string, int> id_map;
};

} // namespace aeplanner

#endif // AGENT_EXPLORATION_MAP_H