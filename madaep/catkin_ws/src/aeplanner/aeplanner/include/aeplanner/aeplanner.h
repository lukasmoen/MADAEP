#ifndef AEPLANNER_H
#define AEPLANNER_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <kdtree/kdtree.h>
#include <sensor_msgs/PointCloud2.h>

#include <aeplanner/data_structures.h>
#include <aeplanner/param.h>
#include <aeplanner/Reevaluate.h>

#include <aeplanner/aeplanner_viz.h>
#include <visualization_msgs/MarkerArray.h>

#include <aeplanner/aeplannerAction.h>
#include <actionlib/server/simple_action_server.h>

#include <pigain/Node.h>
#include <pigain/Query.h>
#include <pigain/BestNode.h>

#include <std_msgs/Bool.h>
#include <aeplanner/Goals.h>
// SDF map, pcl cloud
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_env/raycast.h>
#include "plan_env/map_ros.h"

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// DAEP
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <aeplanner/kalman.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <iostream>
#include <tuple>
#include <pigain/QueryDFM.h>
#include <pigain/Score.h>

// synch
#include <mutex>
#include <aeplanner/agentExplorationMap.h>

namespace aeplanner
{
class AEPlanner
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<aeplanner::aeplannerAction> as_;

  Params params_;

  // Current state of agent (x, y, z, yaw)
  Eigen::Vector4d current_state_;
  bool current_state_initialized_;
  bool dynamic_mode_;
  bool sdf_map_updated;
  bool updated;

  // Keep track of the best node and its score
  RRTNode* best_node_;
  RRTNode* best_branch_root_;

  std::shared_ptr<octomap::OcTree> ot_;

  // kd tree for finding nearest neighbours
  kdtree* kd_tree_;

  // SDF MAP
  fast_planner::SDFMap* sdf_map_;
  double visualization_truncate_height_, visualization_truncate_low_;

  // Subscribers
  ros::Subscriber octomap_sub_;
  ros::Subscriber agent_pose_sub_;
  ros::Subscriber sdf_map_sub_;


  // DAEP
  ros::Subscriber human_sub_;
  ros::Publisher pred_marker_pub_;
  ros::Publisher human_marker_pub_;
  ros::Publisher covariance_marker_pub_;
  ros::Publisher best_marker_pub_;
  ros::Publisher ghost_marker_pub_;
  ros::Publisher static_rays_marker_pub_;
  ros::Publisher dynamic_rays_marker_pub_;
  ros::Publisher old_path_marker_pub_;
  // Publishers
  ros::Publisher rrt_marker_pub_;
  ros::Publisher gain_pub_;
  ros::Publisher sdf_map_pub_;
  ros::Publisher aeplanner_init_pub_;

  // Services
  ros::ServiceClient best_node_client_;
  ros::ServiceClient gp_query_client_;
  ros::ServiceServer reevaluate_server_;
  ros::ServiceClient nn_yaw_query_client_;
  ros::ServiceClient dfm_client_;


  AgentExplorationMap* exploration_map_;

  // DAEP

  //Private variables
  std::string _model_name; 

  // voronoi 
  std::map<std::string, geometry_msgs::Pose> drone_positions;

  //Global variables
  std::map<std::string, std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> dynamic_objects;
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data;
  std::mutex vecMutex;

  // voronoi helpers
  std::vector<RRTNode*> best_nodes_; 

  std::vector<std::tuple<double, double, Eigen::MatrixXd>> createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices);
  bool willViewBeBlocked(Eigen::Vector3d point, int index, bool visualize_ghosts);

  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> KFpredictTrajectories();
  int getCovarianceIndex(double max_time_step, double time_step, double t);
  bool checkCollision(double t, 
                      Eigen::Vector4d point, 
                      std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                      std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses);

  bool isCircleInsideEllipse(const Eigen::Vector3d& point, const Eigen::Vector3d& center, 
            std::tuple<double, double, Eigen::MatrixXd> covariance_ellipse);

  bool isCollisionWithBoundingBox(Eigen::Vector3d point, double x, double y, double z);

  // Service server callback
  bool reevaluate(aeplanner::Reevaluate::Request& req,
                  aeplanner::Reevaluate::Response& res);

  // ---------------- Initialization ----------------
  RRTNode* initialize(std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                              std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses);
                              void initializeKDTreeWithPreviousBestBranch(RRTNode* root);

  std::pair<RRTNode*, bool> pathIsSafe(RRTNode* node, 
                          std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                          std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses);
  void reevaluatePotentialInformationGainRecursive(RRTNode* node);

  // ---------------- Expand RRT Tree ----------------
  void expandRRT(std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                          std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses);

  Eigen::Vector4d sampleNewPoint();
  bool isInsideBoundaries(Eigen::Vector4d point);
  bool isInsideVoronoi(Eigen::Vector4d point);
  bool isClosestAgent(Eigen::Vector4d point);
  bool pointOnXYBoundaries(Eigen::Vector4d point);
  bool collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r);
  RRTNode* chooseParent(RRTNode* node, double l);
  void rewire(kdtree* kd_tree, RRTNode* new_node, double l, double r, double r_os);
  Eigen::Vector4d restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos);

  std::tuple<double, double, double> getGain(RRTNode* node, double time_of_arrival);

  std::tuple<double, double, double> gainCubature(Eigen::Vector4d state, double time_of_arrival);

  // ---------------- Helpers ----------------
  //
  void publishEvaluatedNodesRecursive(RRTNode* node);

  geometry_msgs::Pose vecToPose(Eigen::Vector4d state);

  float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                          float lsq, float rsq, const octomap::point3d& pt);

  void publishESDFMap();

  // ---------------- Frontier ----------------
  geometry_msgs::PoseArray getFrontiers();

  std::map<std::string, aeplanner::Goals> agent_goals_map_; 

public:
  AEPlanner(ros::NodeHandle& nh);

  void execute(const aeplanner::aeplannerGoalConstPtr& goal);

  // DAEP
  void updateHumanPositions(const gazebo_msgs::ModelStates& model_states);

  void octomapCallback(const octomap_msgs::Octomap& msg);
  void agentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void sdfMapCallback(const sensor_msgs::PointCloud2& msg);


  Eigen::Vector4d closestAgent(Eigen::Vector4d point);
  std::map<std::string, ros::Time> drone_stationary_since;
  std::map<std::string, geometry_msgs::Pose> previous_drone_positions;
  std::map<std::string, bool> exploration_completed;
  std::map<std::string, ros::Subscriber> exploration_status_subscribers;

  void explorationStatusCallback(const std_msgs::Bool::ConstPtr& msg, const std::string& drone_id);
  void registerDroneForTracking(const std::string& drone_id);
  void updateDroneStationaryStatus();


  // Vector to store drone goal subscribers
  std::vector<ros::Subscriber> other_goal_subs_;
  std::vector<aeplanner::Goals> other_goals_;
  std::string self_ns_;
  // Callback for drone goals
  void setupOtherGoalSubscribers(int drone_num);
  void otherGoalCallback(const aeplanner::Goals::ConstPtr& msg, const std::string& agent_ns);
  bool isPointInGoalFOV(const Eigen::Vector3d& point, const aeplanner::Goal& goal, double fov_angle_rad);

};

}  // namespace aeplanner

#endif
