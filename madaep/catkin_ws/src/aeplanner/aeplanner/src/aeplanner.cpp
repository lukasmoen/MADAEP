#include <aeplanner/aeplanner.h>
#include <tf2/utils.h>
#include <limits>
ros::Time last_print_time = ros::Time(0); 

namespace aeplanner
{
AEPlanner::AEPlanner(ros::NodeHandle& nh)
  : nh_(nh)
  , as_(nh_, "make_plan", boost::bind(&AEPlanner::execute, this, _1), false)
  , octomap_sub_(nh_.subscribe("octomap", 1, &AEPlanner::octomapCallback, this))
  , agent_pose_sub_(nh_.subscribe("/pose", 1, &AEPlanner::agentPoseCallback, this))
  , human_sub_(nh_.subscribe("/gazebo/model_states", 1, &AEPlanner::updateHumanPositions, this)) // DAEP
  , rrt_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("rrtree", 1000))
  , pred_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("predicted_trajectory", 1000)) // DAEP
  , human_marker_pub_(nh_.advertise<visualization_msgs::Marker>("human_poses", 1000)) // DAEP
  , best_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("best_node", 1000)) // DAEP
  , covariance_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("cov_ellipse", 1000)) // DAEP
  , ghost_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("ghosts", 1000)) // DAEP
  , static_rays_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("static_rays", 1000)) // DAEP
  , dynamic_rays_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("dynamic_rays", 1000)) // DAEP
  , old_path_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("old_path", 1000)) // DAEP
  , gain_pub_(nh_.advertise<pigain::Node>("gain_node", 1000))
  , gp_query_client_(nh_.serviceClient<pigain::Query>("gp_query_server"))
  , nn_yaw_query_client_(nh_.serviceClient<pigain::Query>("nn_yaw_query_server"))
  , reevaluate_server_(nh_.advertiseService("reevaluate", &AEPlanner::reevaluate, this))
  , best_node_client_(nh_.serviceClient<pigain::BestNode>("best_node_server"))
  , dfm_client_(nh_.serviceClient<pigain::QueryDFM>("dfm_query_server"))
  , current_state_initialized_(false)
  , ot_(NULL)
  , best_node_(NULL)
  , best_branch_root_(NULL)
  , updated(false)
  , dynamic_mode_(false)
  , sdf_map_sub_(nh_.subscribe("/sdf_map/occupancy_all_1", 1, &AEPlanner::sdfMapCallback, this))
  , sdf_map_pub_(nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_pub", 100))
  , aeplanner_init_pub_(nh_.advertise<std_msgs::Bool>("/aeplanner_init", 1))

{
  // Use a map to store goals by agent namespace
  // agent_goals_map_ = std::map<std::string, aeplanner::Goals>();  

  // Create agent pose subscriber
  agent_pose_sub_ = nh_.subscribe("/pose", 1, &AEPlanner::agentPoseCallback, this);

  // Read the number of drones from rosparam
  int drone_num = 1;
  if (nh_.hasParam("/drone_num")) {
    nh_.getParam("/drone_num", drone_num);
    ROS_INFO("[AEP] Found drone_num parameter: %d", drone_num);
  } else {
    ROS_WARN("[AEP] drone_num parameter not found, defaulting to 1");
  }
  
  self_ns_ = ros::this_node::getNamespace();
  setupOtherGoalSubscribers(drone_num);

  ROS_INFO("[AEP] Initializing SDF Map...");
    try {
      sdf_map_ = new fast_planner::SDFMap();
      sdf_map_->initMap(nh);
      ROS_INFO("[AEP] SDF Map Initialized Successfully.");
  } catch (const std::exception& e) {
      ROS_ERROR("Failed to initialize SDF Map: %s", e.what());
  }

  params_ = readParams();
  as_.start();
  kd_tree_ = kd_create(3);

  // multi agent gaussian view
  exploration_map_ = new AgentExplorationMap(nh_);

  // Initialize drone tracking variables
  for (int id = 1; id <= drone_num; ++id) {
  std::string drone_ns = "/drone" + std::to_string(id);
  
  // Skip if this is our own namespace
  if (drone_ns == self_ns_)
    continue;
    
  // Register this drone for exploration status tracking
  registerDroneForTracking(drone_ns);
}
}

bool AEPlanner::isClosestAgent(Eigen::Vector4d point) {
  std::string my_name = ros::this_node::getNamespace();
  std::string closest_name;
  double min_dist = std::numeric_limits<double>::max();

  // ROS_ERROR_STREAM("Checking closest agent to point (" 
                  //  << point[0] << ", " << point[1] << ", " << point[2] << ") from " << my_name);

  for (const auto& kv : drone_positions) {
      const std::string& name = kv.first;
      const geometry_msgs::Pose& pose = kv.second;

      Eigen::Vector3d pos(pose.position.x, pose.position.y, pose.position.z);
      double dist = (pos - point.head<3>()).norm();

      // ROS_ERROR_STREAM("Agent: " << name << ", Distance to point: " << dist);

      if (dist < min_dist) {
          min_dist = dist;
          closest_name = name;
      }
  }

  // ROS_ERROR_STREAM("Closest agent: " << closest_name);
  return (closest_name == my_name);
}


bool AEPlanner::isInsideVoronoi(Eigen::Vector4d point) {
  if (isClosestAgent(point)) {
      // ROS_ERROR_STREAM("This agent is the closest. Inside Voronoi.");
      return true;
  }

  Eigen::Vector4d other_agent = closestAgent(point);
  // ROS_ERROR_STREAM("Another agent is closer. Checking collision to it...");

  if (collisionLine(other_agent, point, params_.bounding_radius)) {
      // ROS_ERROR_STREAM("Path from point to closest agent is occluded. Inside Voronoi.");
      return true;
  }

  // ROS_ERROR_STREAM("Path to closest agent is clear. Outside Voronoi.");
  return false;
}


void AEPlanner::registerDroneForTracking(const std::string& drone_id) {
  
  // Skip if already registered
  if (exploration_status_subscribers.find(drone_id) != exploration_status_subscribers.end()) {
    return;
  }
  
  // Subscribe to exploration completion topic
  std::string exploration_topic = drone_id + "/exploration_completed";
  exploration_status_subscribers[drone_id] = nh_.subscribe<std_msgs::Bool>(
      exploration_topic, 1, boost::bind(&AEPlanner::explorationStatusCallback, this, _1, drone_id));
  
  // Initialize exploration status to false
  exploration_completed[drone_id] = false;
  
}

void AEPlanner::explorationStatusCallback(const std_msgs::Bool::ConstPtr& msg, const std::string& drone_id) {
  // Update exploration status
  exploration_completed[drone_id] = msg->data;
  
  if (msg->data) {
    ROS_INFO("[AEP] Drone %s has completed exploration", drone_id.c_str());
  }
}

void AEPlanner::updateDroneStationaryStatus() {
  // Current time
  ros::Time current_time = ros::Time::now();
  
  // For each drone position we know about
  for (auto it = drone_positions.begin(); it != drone_positions.end(); ++it) {
    const std::string& drone_id = it->first;
    const geometry_msgs::Pose& current_pose = it->second;
    
    // Register drone for tracking if we haven't already
    registerDroneForTracking(drone_id);
    
    // Check if this is the first position update
    if (previous_drone_positions.find(drone_id) == previous_drone_positions.end()) {
      previous_drone_positions[drone_id] = current_pose;
      continue;
    }
    
    // Calculate distance moved since last position
    const geometry_msgs::Pose& prev_pose = previous_drone_positions[drone_id];
    
    double dx = current_pose.position.x - prev_pose.position.x;
    double dy = current_pose.position.y - prev_pose.position.y;
    double dz = current_pose.position.z - prev_pose.position.z;
    
    double distance_moved = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // Update previous position
    previous_drone_positions[drone_id] = current_pose;
    
    // Movement threshold in meters
    const double movement_threshold = 0.05; // 5cm
    
    // If drone has moved more than threshold, update stationary time
    if (distance_moved > movement_threshold) {
      // Drone is moving, remove it from stationary map if present
      drone_stationary_since.erase(drone_id);
    } else {
      // Drone is stationary, record time if not already recorded
      if (drone_stationary_since.find(drone_id) == drone_stationary_since.end()) {
        drone_stationary_since[drone_id] = current_time;
      }
      // Time continues to be the first time it became stationary
    }
  }
}

void AEPlanner::setupOtherGoalSubscribers(int drone_num) {
  ros::NodeHandle nh;

  for (int id = 1; id <= drone_num; ++id) {
    std::string drone_ns = "/drone" + std::to_string(id);
    if (drone_ns == self_ns_)
      continue;  // Skip subscribing to myself

    std::string topic = drone_ns + "/goal";
    ROS_INFO_STREAM("[AEP] Subscribing to " << topic);

    // Create a bound function with the drone namespace as a parameter
    ros::Subscriber sub = nh.subscribe<aeplanner::Goals>(
      topic,
      10,
      boost::bind(&AEPlanner::otherGoalCallback, this, _1, drone_ns)
    );
    other_goal_subs_.push_back(sub);
  }
}

// Modified callback to include agent information
void AEPlanner::otherGoalCallback(const aeplanner::Goals::ConstPtr& msg, const std::string& agent_ns) {
  // Store the goals for this specific agent
  // agent_goals_map_[agent_ns] = *msg;
  
  // ROS_INFO_STREAM("[AEP] Received goals from " << agent_ns << " with ID: " << msg->id);
  
  // // Pass only this agent's goals to the exploration map
  std::vector<aeplanner::Goals> single_agent_goals;
  single_agent_goals.push_back(*msg);

  if (exploration_map_){
    exploration_map_->addGoalsAsGaussians(single_agent_goals, msg->id, agent_ns);
  }
}

// // Helper method to get all goals from all agents
// std::vector<aeplanner::Goals> AEPlanner::getAllAgentGoals() {
//   std::vector<aeplanner::Goals> all_goals;
//   for (const auto& pair : agent_goals_map_) {
//     all_goals.push_back(pair.second);
//   }
//   return all_goals;
// }

// // Helper method to get goals for a specific agent
// bool AEPlanner::getAgentGoals(const std::string& agent_ns, aeplanner::Goals& goals) {
//   auto it = agent_goals_map_.find(agent_ns);
//   if (it != agent_goals_map_.end()) {
//     goals = it->second;
//     return true;
//   }
//   return false;
// }


void AEPlanner::sdfMapCallback(const sensor_msgs::PointCloud2& msg)
{
  if (!sdf_map_) {
    ROS_ERROR("SDF Map is not initialized!");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(msg, pcl_cloud);
  
  if (pcl_cloud.empty()) {
    ROS_WARN("[AEP] Received empty point cloud in sdfMapCallback!");
    return;
  }

  int point_num = pcl_cloud.size();
  Eigen::Vector3d origin(current_state_[0], current_state_[1], current_state_[2]);

  // Manually mark occupied points
  for (const auto& pt : pcl_cloud.points) {
    Eigen::Vector3d obs_pt(pt.x, pt.y, pt.z);
    sdf_map_->setOccupied(obs_pt);  // Explicitly set occupied
  }

  sdf_map_->inputPointCloud(pcl_cloud, point_num, origin);
  sdf_map_->updateESDF3d(); // Recalculate the signed distance field

  sdf_map_updated = true;
}


// Helper function for debugging
// void AEPlanner::publishESDFMap() {
//   pcl::PointXYZ pt;
//   pcl::PointCloud<pcl::PointXYZ> cloud1;

//   Eigen::Vector3i min_idx, max_idx;
//   sdf_map_->posToIndex(sdf_map_->md_->all_min_, min_idx);
//   sdf_map_->posToIndex(sdf_map_->md_->all_max_, max_idx);

//   sdf_map_->boundIndex(min_idx);
//   sdf_map_->boundIndex(max_idx);
  
//   for (int x = min_idx[0]; x <= max_idx[0]; ++x)
//     for (int y = min_idx[1]; y <= max_idx[1]; ++y)
//       for (int z = min_idx[2]; z <= max_idx[2]; ++z) {
//         if (sdf_map_->md_->occupancy_buffer_[sdf_map_->toAddress(x, y, z)] >
//           sdf_map_->mp_->min_occupancy_log_) {
//           Eigen::Vector3d pos;
//           sdf_map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
//           if (pos(2) > visualization_truncate_height_) continue;
//           if (pos(2) < visualization_truncate_low_) continue;
//           pt.x = pos(0);
//           pt.y = pos(1);
//           pt.z = pos(2);
//           cloud1.push_back(pt);
//         }
//       }
//   cloud1.width = cloud1.points.size();
//   cloud1.height = 1;
//   cloud1.is_dense = true;
//   cloud1.header.frame_id = "world";
//   sensor_msgs::PointCloud2 cloud_msg;
//   pcl::toROSMsg(cloud1, cloud_msg);
//   sdf_map_pub_.publish(cloud_msg);
// }

/**
 * Subscribe on the gazebo/model_states topic to extract the positions of the 
 * dynamic obstacles. Add these to the dynamic_objects std::map and visualzie 
 * the ground truth in RViz.
*/
void AEPlanner::updateHumanPositions(const gazebo_msgs::ModelStates& model_states) {
    int human_id = 0;

    for (size_t i = 0; i < model_states.name.size(); ++i) {
        if (model_states.name[i].find("person_walking") != std::string::npos) {
            dynamic_mode_ = true;
            std::string model_name = model_states.name[i];
            geometry_msgs::Pose person_pose = model_states.pose[i];
            geometry_msgs::Twist person_twist = model_states.twist[i];
            dynamic_objects[model_name] = std::make_pair(person_pose, person_twist);
            visualizePose(human_marker_pub_, human_id, person_pose);
        }
        human_id++;
        // Check for drones
        if (model_states.name[i].find("drone") != std::string::npos) {
          std::string model_name = "/"+model_states.name[i];
          drone_positions[model_name] = model_states.pose[i];
      }
    }
    updateDroneStationaryStatus();
} 

/**
 * From a vector of covariance matrices, compute a list of tuples that
 * represent each covariance circle.
 * 
 * Return: (major_lenght, minor_length, eigenvectors) 
*/
std::vector<std::tuple<double, double, Eigen::MatrixXd>> AEPlanner::createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices)
{
  
  std::vector<std::tuple<double, double, Eigen::MatrixXd>> ellipses{};

  for(auto const& cov_matrix : cov_matrices)
  {
    //Extract the part concerning x,y
    Eigen::MatrixXd cov_matrix_xy = cov_matrix.block<2,2>(0,0);

    // Compute the eigenvalues and eigenvectors of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov_matrix_xy);
    Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
    Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();

    // Compute the length of the major and minor axes of the ellipse
    double major_length = std::sqrt(std::max(eigenvalues(0), eigenvalues(1)));
    double minor_length = std::sqrt(std::min(eigenvalues(0), eigenvalues(1)));

    ellipses.push_back(std::make_tuple(major_length, minor_length, eigenvectors));
  }
  return ellipses;
}


/**
* This function uses the Kalman Filter with the Constant Velocity
* motion model to predict the future trajectory of each dynamic obstacle.
* The returned data consists of the means and covariances for each dynamic obstacle in a trajectory.
*/
std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> AEPlanner::KFpredictTrajectories()
{
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> kf_data{};

  //For each pedestrian, we need to build a new Kalman filter
  int n = 4; //Number of states
  int m = 2; //Number of states we measure

  double dt = params_.time_step;

  //Matrices for the Kalman filter
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  //Constant Velocity Model
  A <<  1, 0, dt, 0, 
        0, 1, 0, dt, 
        0, 0, 1, 0, 
        0, 0, 0, 1;

  C << 1, 0, 0, 0, 0, 1, 0, 0;

  //Reasonable covariance matrices
  Q << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;
  
  R << 0.01, 0,
       0, 0.01;
  
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1; 

  Eigen::VectorXd y(m);
  Eigen::VectorXd x0(n);
  Eigen::MatrixXd P0(n, n);
  double t = 0;
  int safety_iteration = 1; // Bounding covariance (1 second)
  
  for (const auto& dynamic_obstacle : dynamic_objects) {
      const std::string& key = dynamic_obstacle.first;
      const geometry_msgs::Pose& pose = dynamic_obstacle.second.first;
      const geometry_msgs::Twist& twist = dynamic_obstacle.second.second;

      // Create a new Kalman filter for each dynamic obstacle
      KalmanFilter kf(dt, A, C, Q, R, P);
      std::vector<double> xcoords{};
      std::vector<double> ycoords{};
      std::vector<double> zcoords{};
      std::vector<Eigen::MatrixXd> covariance_matrices(params_.KFiterations);

      // Initalize with the last measurement
      double xcoord = pose.position.x, ycoord = pose.position.y, zcoord = pose.position.z;
      double vx = twist.linear.x, vy = twist.linear.y;
      x0 << xcoord, ycoord, vx, vy;
      
      //Initial covariance
      P0 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; 

      kf.init(t, x0, P0);

      //Perform one round of the Kalman filter
      y << xcoord, ycoord;
      kf.predict();
      kf.update(y);

      //Create the trajectory for the current dynamic obstacle
      Eigen::MatrixXd safety_margin(n, n);
      
      for (int i = 0; i < params_.KFiterations; i++)
      {
        kf.predict();
        xcoords.push_back(kf.state()(0));
        ycoords.push_back(kf.state()(1));
        zcoords.push_back(zcoord);
        if(i == safety_iteration)
        {
          //Because of controllability we can only extract one of these
          //ellipses. See paper for explanation.
          safety_margin = kf.covariance();
        }
      }

      std::fill(covariance_matrices.begin(), covariance_matrices.end(), safety_margin);  
      kf_data.push_back(std::make_pair(std::make_tuple(xcoords, ycoords, zcoords), covariance_matrices)); 
  }
    return kf_data;
}

/*
* Gets the correct covariance ellipse from a prediction given time t.
*/
int AEPlanner::getCovarianceIndex(double max_time_step, double time_step, double t){
  
  int num_steps = static_cast<int>(max_time_step / time_step); // Calculate the number of time steps

  if(t > max_time_step)
  {
    //No prediction available
    return -1;
  }

  // Initialize the minimum difference and index
  double min_difference = std::abs(t - time_step);
  int index = 0;

  // Loop through each time step and find the closest one
  for (int i = 1; i < num_steps; ++i) {
    double time = i * time_step; // Calculate the time for the current step
    double difference = std::abs(t - time);
    if (difference < min_difference) {
      min_difference = difference;
      index = i;
    }
  }
  return index;
}

/*
* Check intersection between a circle around a goal and the predicted covariance ellipse (currently implemented for a circle).
*/
bool AEPlanner::isCircleInsideEllipse(const Eigen::Vector3d& point, const Eigen::Vector3d& center, 
            std::tuple<double, double, Eigen::MatrixXd> covariance_ellipse) 
{ 
  /*
  // Radius around goal
  double radius = 0.2;

  // Extract major and minor axis lengths and eigenvectors
  double major_axis_length = std::get<0>(covariance_ellipse);
  double minor_axis_length = std::get<1>(covariance_ellipse);
  Eigen::MatrixXd eigenvectors = std::get<2>(covariance_ellipse);
  
  // Calculate relative coordinates of the point with respect to the ellipse center (XY)
  Eigen::Vector2d relative_point = point.head(2) - center.head(2);

  // Rotate the relative coordinates by the angle of the major axis
  Eigen::Vector2d rotated_relative_point = eigenvectors.transpose() * relative_point;

  // Normalize the rotated relative coordinates
  double normalized_x = rotated_relative_point(0) / major_axis_length;
  double normalized_y = rotated_relative_point(1) / minor_axis_length;

  // Check if the normalized coordinates are inside the unit circle
  double distance_from_origin = normalized_x * normalized_x + normalized_y * normalized_y;
  
  // Calculate the normalized radius of the circle
  double normalized_radius = radius / std::max(major_axis_length, minor_axis_length);

  // Check if the circle intersects with the ellipse
  bool does_intersect_XY = distance_from_origin <= 1.0 + normalized_radius;
  // Check if the sphere intersects with the cylinder
  bool does_intersect_XYZ = does_intersect_XY and point(2) < center(2) + params_.human_height + (2*params_.drone_height);
  return does_intersect_XYZ;
  */
  double circle_radius = std::get<0>(covariance_ellipse);
  // Calculate the distance between the centers of the two circles
  Eigen::Vector2d point_XY = point.head(2);
  double distance = (point_XY - center.head(2)).norm();

  // Check if the two circles intersect
  bool does_intersect_XY = distance <= 0.2 + circle_radius;
  bool does_intersect_XYZ = does_intersect_XY and point(2) < center(2) + params_.human_height + (2*params_.drone_height);

  return does_intersect_XYZ;

}


/* 
* Calculate if there is a potential collision with any moving obstacle in time t.
*/
bool AEPlanner::checkCollision(double t, 
                              Eigen::Vector4d point, 
                              std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                              std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses)
{
  double max_time_step = params_.KFiterations * params_.time_step;
  int N_persons = trajectories.size();
  
  // Index the correct circle
  int covariance_index = getCovarianceIndex(max_time_step, params_.time_step, t);

  if(covariance_index == -1)
  { 
    // Outside our prediction
    return false;
  }

  for(int i = 0; i < N_persons; i++)
  { 
      
      auto personEllipses = covarianceEllipses[i];
      auto personTrajectory = trajectories[i];
      
      //Extract the correct mean and ellipse
      std::tuple<double, double, Eigen::MatrixXd> covarianceEllipse = personEllipses[covariance_index];
      
      double ellipse_center_x = std::get<0>(personTrajectory)[covariance_index];
      double ellipse_center_y = std::get<1>(personTrajectory)[covariance_index];
      double ellipse_center_z = std::get<2>(personTrajectory)[covariance_index];

      double person_current_x = std::get<0>(personTrajectory)[0];
      double person_current_y = std::get<1>(personTrajectory)[0];

      //if(sqrt(pow(person_current_x-point[0], 2.0) + pow(person_current_y-point[1], 2.0)) < params_.KFiterations * params_.time_step * params_.human_linear_velocity)
      //{
        // Check if person is close enough
        const Eigen::Vector3d center(ellipse_center_x, ellipse_center_y, ellipse_center_z);
        
        if(isCircleInsideEllipse(point.head(3), center, covarianceEllipse)) {
          return true;
        }
      //}
  }
  return false;
}


void AEPlanner::execute(const aeplanner::aeplannerGoalConstPtr& goal)
{
  aeplanner::aeplannerResult result;

  // ros::Time current_time = ros::Time::now();
  // if (!last_print_time.isZero())
  // {
  //     double dt = (current_time - last_print_time).toSec();
  //     ROS_INFO_STREAM("Time since last print: " << dt << " seconds");
  // }
  // last_print_time = current_time;

  // Check if aeplanner has recieved agent's pose yet
  if (!current_state_initialized_)
  {
    ROS_WARN("Agent's pose not yet received");
    ROS_WARN("Make sure it is being published and correctly mapped");
    as_.setSucceeded(result);
    return;
  }
  // if (!ot_)
  // {
  //   ROS_WARN("No octomap received");
  //   as_.setSucceeded(result);
  //   return;
  // }
  if (!sdf_map_updated) {
    ROS_ERROR("SDF Map is not initialized!");
    as_.setSucceeded(result);
    return;
  }

  // Perform prediction of human obstacles future positions 
  // using the Kalman Filter with Constant Velocity Model
  auto new_vec = KFpredictTrajectories();
  { 
    // Race condition
    std::lock_guard<std::mutex> lock(vecMutex);
    predicted_data = new_vec;
  }

  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

  // Separate the covariance ellipses and the trajectories.
  for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  // Visualize covariance and predicted trajectory
  visualizePrediction(pred_marker_pub_, covariance_marker_pub_, trajectories, all_ellipses);

  RRTNode* root = initialize(trajectories, all_ellipses);

  // Check if we have a old best branch
  if (best_branch_root_)
  {
    // Check if previous branch is collision free
    std::pair<RRTNode*, bool> result = pathIsSafe(best_branch_root_, trajectories, all_ellipses);
    if(result.second)
    {
      // Get candidate goal
      expandRRT(trajectories, all_ellipses);

      // Check if we got a valid candidate node
      if(best_node_)
      {
        // Compare candidate goal with previous goal
        double old_score = result.first->dynamic_score(params_.lambda, params_.zeta);
        double new_score = best_node_->dynamic_score(params_.lambda, params_.zeta);
        
        // if new goal is less than 10% better, choose old goal
        // if(new_score/old_score < 1.1){
        //   ROS_INFO_STREAM("old goal better");
        //   best_node_ = result.first;
        // }
        // else {ROS_INFO_STREAM("new goal better");}
      }
    }
    else 
    {
      // Old branch contains collision
      expandRRT(trajectories, all_ellipses);
    }
  }
  else 
  {
    //No best branch available
    expandRRT(trajectories, all_ellipses);
  }

  // No valid nodes found in expandRRT
  if (best_node_ == NULL){
    // we cant find a valid node, --> frontier planning
    ROS_DEBUG_STREAM("GLOBAL PLANNING");
    result.frontiers = getFrontiers();
    result.is_clear = false;
    as_.setSucceeded(result);
    ROS_DEBUG("Deleting/Freeing!");
    delete root;
    kd_free(kd_tree_);
    ROS_DEBUG("Done!");
    return;
  }

  ROS_DEBUG_STREAM("getCopyOfParent");
  best_branch_root_ = best_node_->getCopyOfParentBranch();

  ROS_DEBUG_STREAM("createRRTMarker");
  rrt_marker_pub_.publish(createRRTMarkerArray(root, params_.lambda));
  ROS_DEBUG_STREAM("publishRecursive");
  publishEvaluatedNodesRecursive(root);

  RRTNode* last_node = best_branch_root_;
  while (!last_node->children_.empty()) {
      last_node = last_node->children_.back();  // Move to the last child
  }
 

  // OPTION 1, FIRST NODE
  // Only one action along best branch is executed
  result.pose.pose = vecToPose(best_branch_root_->children_[0]->state_);

  // OPTION 2, LAST NODE
  // Find the last node in the best branch
  // RRTNode* last_node = best_branch_root_;
  // while (!last_node->children_.empty()) {
  //     last_node = last_node->children_.back();  // Move to the last child
  // }
  // result.pose.pose = vecToPose(last_node->state_);

  // OPTION 3, ALL NODES IN BRANCH (not complete)
  // Send all nodes and let motion planner generate path
 
  geometry_msgs::PoseArray poses;
  RRTNode* current_node = best_branch_root_;
  while (!current_node->children_.empty()) {
    RRTNode* next_node = current_node->children_.back();  // Next in best branch
    poses.poses.push_back(vecToPose(current_node->state_));
    current_node = next_node;
}

  // Add the last node
  poses.poses.push_back(vecToPose(current_node->state_));

  result.poses = poses;

  // If we find a best node
  if (best_node_->dynamic_score(params_.lambda, params_.zeta) > params_.zero_gain)
  {
    result.is_clear = true;
    ROS_DEBUG_STREAM("LOCAL PLANNING");
  }
  else
  {
    // we cant find a valid node, --> frontier planning
    ROS_DEBUG_STREAM("GLOBAL PLANNING");
    result.frontiers = getFrontiers();
    result.is_clear = false;
    delete best_branch_root_;
    best_branch_root_ = NULL;
  }

  as_.setSucceeded(result);
  ROS_DEBUG_STREAM("Deleting/Freeing!");
  delete root;
  kd_free(kd_tree_);
  ROS_DEBUG_STREAM("Done!");
}


RRTNode* AEPlanner::initialize(std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                               std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses)
{
  // Initialize kd-tree
  /* create a kd-tree for 3-dimensional data */
  kd_tree_ = kd_create(3);
  best_node_ = NULL;
  RRTNode* root = NULL;
  if(best_branch_root_){
    reevaluatePotentialInformationGainRecursive(best_branch_root_);
    best_branch_root_ = best_branch_root_->children_[0];
    VisualizeOldPath(old_path_marker_pub_, best_branch_root_);

    ROS_DEBUG_STREAM("best branch root : (" << best_branch_root_->state_(0) << ", " << best_branch_root_->state_(1) << ", " << best_branch_root_->state_(2) << ", " << best_branch_root_->state_(3) << ")");

    if(best_branch_root_->children_.size() > 0){
        best_branch_root_->parent_ = NULL;
    }
    else{
      delete best_branch_root_;
      best_branch_root_ = NULL;
    }
  }
  

  // Initialize without any previous branch
  root = new RRTNode();
  root->state_[0] = current_state_[0];
  root->state_[1] = current_state_[1];
  root->state_[2] = current_state_[2];
  ROS_DEBUG_STREAM("kd_insert3 x: " << current_state_[0] << " y: "<< current_state_[1] << " z: "<< current_state_[2] << " yaw: "<< current_state_[3]);
  kd_insert3(kd_tree_, root->state_[0], root->state_[1], root->state_[2], root);
  return root;
}


/*
Check if a branch is predicted to be collision free
*/
std::pair<RRTNode*, bool> AEPlanner::pathIsSafe(RRTNode* node, 
                                                std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                                                std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses)
{
  if(node == NULL){
    return std::make_pair(nullptr, false);
  }

  RRTNode* curr = node;
  // Traverse until root
  while(!curr->children_.empty()){
    // if any node has a potential collision return false
    double time_to_reach_node = curr->time_cost();

    bool collision = checkCollision(time_to_reach_node, curr->state_, trajectories, covarianceEllipses) && dynamic_mode_;
    if (collision){
      return std::make_pair(nullptr, false);  // Collision detected, return false
    }
    else{
      curr = curr->children_[0];
    }
  }  
  return std::make_pair(curr, true);  // No collision, return the leaf node and true
}

void AEPlanner::reevaluatePotentialInformationGainRecursive(RRTNode* node)
{

 std::tuple<double, double, double> ret = gainCubature(node->state_, node->time_cost());
  node->gain_ = std::get<0>(ret); // Assign static gain
  node->dynamic_gain_ = std::get<1>(ret); //Assign dynamic gain
  node->state_[3] = std::get<2>(ret); // Assign yaw angle that maximizes static gain

  //Acquire DFM score
  pigain::QueryDFMRequest req;
  req.point.x = node->state_[0];
  req.point.y = node->state_[1];
  req.point.z = node->state_[2];
  pigain::QueryDFMResponse res;
  if(dfm_client_.call(req, res))
  {
    node->dfm_score_ = res.score; 
  }


  for (typename std::vector<RRTNode*>::iterator node_it = node->children_.begin();
       node_it != node->children_.end(); ++node_it)
    reevaluatePotentialInformationGainRecursive(*node_it);
}

Eigen::Vector4d AEPlanner::closestAgent(Eigen::Vector4d point) {
  double min_dist = std::numeric_limits<double>::max();
  Eigen::Vector4d closest;
  std::string closest_name;
  
  // Get current time for checking stationary time
  ros::Time current_time = ros::Time::now();
  
  for (const auto& kv : drone_positions) {
      const std::string& name = kv.first;
      const geometry_msgs::Pose& pose = kv.second;
      
      // Skip agents that have been stationary for more than 1 minute
      if (drone_stationary_since.find(name) != drone_stationary_since.end()) {
          ros::Duration stationary_duration = current_time - drone_stationary_since[name];
          if (stationary_duration.toSec() > 60.0) { // 60 seconds = 1 minute
              // Skip agents that have sent a true message on exploration_completed topic
              if (exploration_completed.find(name) != exploration_completed.end() && 
                !exploration_completed[name]) {
                continue; // Skip this agent
              }
          }
      }

      // Calculate distance to this agent
      Eigen::Vector4d pos;
      pos << pose.position.x, pose.position.y, pose.position.z, 0.0;
      double dist = (pos.head<3>() - point.head<3>()).norm();
      
      // ROS_ERROR_STREAM("Checking agent: " << name << " at distance " << dist);
      if (dist < min_dist) {
          min_dist = dist;
          closest = pos;
          closest_name = name;
      }
  }
  
  // ROS_ERROR_STREAM("Closest agent is: " << closest_name << " at distance " << min_dist);
  return closest;
}

void AEPlanner::expandRRT(std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                          std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses)
{
  // std::shared_ptr<octomap::OcTree> ot = ot_;

  fast_planner::SDFMap* sdf_map = sdf_map_;

  double estimated_yaw = 0.0;

  // Expand an RRT tree and calculate information gain in every node
  ROS_DEBUG_STREAM("Entering expanding RRT");
  std::vector<Eigen::Vector4d> validNodes{};
  std::vector<Eigen::Vector4d> candidateNodes{};
  int N_valid = 0; // Valid sampled nodes counter
  int N_sampled_nodes = 0; 
  ROS_INFO("Entering expanding RRT");
  best_nodes_.clear();

  // (1) Sample until N_valid > init_iterations and total sampled nodes < max_sampled_nodes
  // (2) If more than max_sampled_nodes are sampled: continue until max_sampled_nodes as long as dynamic score is not high enough
  while ((N_valid < params_.init_iterations and N_sampled_nodes < params_.max_sampled_initial_nodes) or 
  ((N_valid > 0 and N_sampled_nodes < params_.max_sampled_nodes and best_node_->dynamic_score(params_.lambda, params_.zeta) < params_.zero_gain) 
  and ros::ok()))

  {
    RRTNode* new_node = new RRTNode();
    RRTNode* nearest;
    // octomap::OcTreeNode* ot_result;
    int occ;
    double dist;
    // Sample new point around agent and check that
    // (1) it is within the boundaries
    // (2) it is in known space
    // (3) the path between the new node and it's parent does not contain any
    // obstacles

    // distance from tree nodes to nearest obstacle
    double tree_distance = 0.3;
    do
    {
      Eigen::Vector4d offset = sampleNewPoint();
      new_node->state_ = current_state_ + offset;
      ROS_DEBUG_STREAM("sample x: " << new_node->state_[0] << " y: "<< new_node->state_[1] << " z: "<< new_node->state_[2]);

      nearest = chooseParent(new_node, params_.extension_range);

      new_node->state_ = restrictDistance(nearest->state_, new_node->state_);
      ROS_DEBUG_STREAM("restrictDistance sample x: " << new_node->state_[0] << " y: "<< new_node->state_[1] << " z: "<< new_node->state_[2]);
      
      ROS_DEBUG_STREAM("Trying node (" << new_node->state_[0] << ", "
                                       << new_node->state_[1] << ", "
                                       << new_node->state_[2] << ")");
      ROS_DEBUG_STREAM("nearest (" << nearest->state_[0] << ", " << nearest->state_[1]
                                       << ", " << nearest->state_[2] << ")");

      occ = sdf_map->getOccupancy(Eigen::Vector3d(new_node->state_[0],new_node->state_[1],new_node->state_[2]));

      dist = sdf_map->getDistance(Eigen::Vector3d(new_node->state_[0],new_node->state_[1],new_node->state_[2]));
      
     
      // unknown = 0, free = 1, obstacle = 2
      if (occ == 2 or occ == 0 or dist < tree_distance) // skip point if unknown, obstacle or too close to obstacle 
        continue;

      if (!isInsideVoronoi(new_node->state_))
        continue;

      ROS_DEBUG_STREAM("ot check done!");
      ROS_DEBUG_STREAM("Inside boundaries?  " << isInsideBoundaries(new_node->state_));
      ROS_DEBUG_STREAM("In known space?     " << occ);
      ROS_DEBUG_STREAM("Collision?          " << collisionLine(nearest->state_, new_node->state_, params_.bounding_radius));

    } while (!isInsideBoundaries(new_node->state_) or 
             (occ == 0)  or
             (occ == 2)  or
             (dist < tree_distance) or
             (!isInsideVoronoi(new_node->state_)) or
            collisionLine(nearest->state_, new_node->state_, params_.bounding_radius));


    pigain::Query srv;
    srv.request.point.x = new_node->state_[0];
    srv.request.point.y = new_node->state_[1];
    srv.request.point.z = new_node->state_[2];

    // Estimate the yaw using nearest neighbor
    if (nn_yaw_query_client_.call(srv))
    {
        estimated_yaw = srv.response.yaw;
    }
    new_node->state_[3] = estimated_yaw;

    // Estimate time to reach node
    double time_to_reach_node = nearest->time_cost() + nearest->time_to_reach(new_node);

    bool collision = checkCollision(time_to_reach_node, new_node->state_, trajectories, covarianceEllipses) && dynamic_mode_;
    if (!collision) 
    {
      ROS_DEBUG_STREAM("Get gain");
      std::tuple<double, double, double> ret = getGain(new_node, time_to_reach_node);
      // Update node
      new_node->gain_ = std::get<0>(ret);
      new_node->dynamic_gain_ = std::get<1>(ret);
      new_node->state_[3] = std::get<2>(ret); //Set new yaw that maximizes gain

      //Aquire DFM score
      pigain::QueryDFMRequest req;
      req.point.x = new_node->state_[0];
      req.point.y = new_node->state_[1];
      req.point.z = new_node->state_[2];
      pigain::QueryDFMResponse res;
      if(dfm_client_.call(req, res))
      {
        new_node->dfm_score_ = res.score; 
      }

      N_valid++;
      // new_node is now ready to be added to tree
      new_node->parent_ = nearest; 
      
      nearest->children_.push_back(new_node);

      // rewire tree with new node
      ROS_DEBUG_STREAM("rewire start");
      rewire(kd_tree_, nearest, params_.extension_range, params_.bounding_radius,
            params_.d_overshoot_);

      ROS_DEBUG_STREAM("Insert into KDTREE");
      kd_insert3(kd_tree_, new_node->state_[0], new_node->state_[1], new_node->state_[2],
                new_node);

      // Update best node
      ROS_DEBUG_STREAM("Update best node");
      if (!best_node_ or
          new_node->dynamic_score(params_.lambda, params_.zeta) > best_node_->dynamic_score(params_.lambda, params_.zeta))
          best_node_ = new_node;
          // ROS_ERROR_STREAM("Dynamic score: " << new_node->dynamic_score(params_.lambda, params_.zeta));
      // ROS_ERROR_STREAM("SCORE: "<<best_node_->dynamic_score(params_.lambda, params_.zeta));
      best_nodes_.push_back(new_node);

      // Estimate time to reach node (Now we know the yaw)
      Eigen::Vector4d p4(new_node->state_[0], new_node->state_[1], new_node->state_[2], int(collision));
      validNodes.push_back(p4);
      ROS_DEBUG_STREAM("iteration Done!");
    } 
    else 
    {
      // Estimate time to reach node (Now we know the yaw)
      Eigen::Vector4d p4(new_node->state_[0], new_node->state_[1], new_node->state_[2], int(collision));
      validNodes.push_back(p4);
      ROS_DEBUG_STREAM("Collision, node not valid !");
    }
    N_sampled_nodes++;
  }

  if(N_valid > 0){
  // Visualize the best node as green (=2)
  Eigen::Vector4d p4(best_node_->state_[0], best_node_->state_[1], best_node_->state_[2], 2);
  validNodes.push_back(p4);
  visualizeBestNode(best_marker_pub_, validNodes);
  }
  else
  {
    ROS_DEBUG_STREAM("No valid nodes found!");
  }
  ROS_DEBUG_STREAM("expandRRT Done!");
}

Eigen::Vector4d AEPlanner::sampleNewPoint()
{
  // Samples one point uniformly over a sphere with a radius of
  // param_.max_sampling_radius
  Eigen::Vector4d point;
  do
  {
    for (int i = 0; i < 3; i++)
      point[i] = params_.max_sampling_radius * 2.0 *
                 (((double)rand()) / ((double)RAND_MAX) - 0.5);
  } while (pow(point[0], 2.0) + pow(point[1], 2.0) + pow(point[2], 2.0) >
           pow(params_.max_sampling_radius, 2.0));

  return point;
}

RRTNode* AEPlanner::chooseParent(RRTNode* node, double l)
{
  // std::shared_ptr<octomap::OcTree> ot = ot_;
  Eigen::Vector4d current_state = current_state_;

  // Find nearest neighbour
  kdres* nearest = kd_nearest_range3(kd_tree_, node->state_[0], node->state_[1],
                                     node->state_[2], l + 0.5); // FIXME why +0.5?

  if (kd_res_size(nearest) <= 0)
    nearest = kd_nearest3(kd_tree_, node->state_[0], node->state_[1], node->state_[2]);
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return NULL;
  }

  RRTNode* node_nn = (RRTNode*)kd_res_item_data(nearest);

  RRTNode* best_node = node_nn;
  double best_node_cost = best_node->cost();
  while (!kd_res_end(nearest))
  {
    node_nn = (RRTNode*)kd_res_item_data(nearest);
    double node_cost = node_nn->cost();
    if (best_node and node_cost < best_node_cost)
    {
      best_node = node_nn;
      best_node_cost = node_cost;
    }

    kd_res_next(nearest);
  }

  kd_res_free(nearest);
  return best_node;
}

void AEPlanner::rewire(kdtree* kd_tree, RRTNode* new_node, double l, double r,
                       double r_os)
{
  // std::shared_ptr<octomap::OcTree> ot = ot_;
  Eigen::Vector4d current_state = current_state_;

  RRTNode* node_nn;
  kdres* nearest = kd_nearest_range3(kd_tree, new_node->state_[0], new_node->state_[1],
                                     new_node->state_[2], l + 0.5); // FIXME why +0.5?
  while (!kd_res_end(nearest))
  {
    node_nn = (RRTNode*)kd_res_item_data(nearest);
    Eigen::Vector3d p1(new_node->state_[0], new_node->state_[1], new_node->state_[2]);
    Eigen::Vector3d p2(node_nn->state_[0], node_nn->state_[1], node_nn->state_[2]);
    if (node_nn->cost() > new_node->cost() + (p1 - p2).norm())
    {
      if (!collisionLine(new_node->state_, node_nn->state_, r))
        node_nn->parent_ = new_node;
    }
    kd_res_next(nearest);
  }
}

Eigen::Vector4d AEPlanner::restrictDistance(Eigen::Vector4d nearest,
                                            Eigen::Vector4d new_pos)
{
  // Check for collision
  Eigen::Vector3d origin(nearest[0], nearest[1], nearest[2]);
  Eigen::Vector3d direction(new_pos[0] - origin[0], new_pos[1] - origin[1],
                            new_pos[2] - origin[2]);
  if (direction.norm() > params_.extension_range)
    direction = params_.extension_range * direction.normalized();

  new_pos[0] = origin[0] + direction[0];
  new_pos[1] = origin[1] + direction[1];
  new_pos[2] = origin[2] + direction[2];

  return new_pos;
}

std::tuple<double, double, double> AEPlanner::getGain(RRTNode* node, double time_of_arrival)
{
  pigain::Query srv;
  srv.request.point.x = node->state_[0];
  srv.request.point.y = node->state_[1];
  srv.request.point.z = node->state_[2];
  
  // GP currently deactivated
  //if (gp_query_client_.call(srv))
  //{
  //  double gain = srv.response.mu;
  //  double yaw = srv.response.yaw;
  //  ROS_DEBUG_STREAM("gain impl: " << gain);
  //  ROS_DEBUG_STREAM("sigma: " << srv.response.sigma);
   // if (srv.response.sigma < params_.sigma_thresh)
   // {
   //   double gain = srv.response.mu;
   //   double yaw = srv.response.yaw;
   //
   //   ROS_DEBUG_STREAM("gain impl: " << gain);
   //   return std::make_pair(gain, yaw);
   // }
  //}
  
  node->gain_explicitly_calculated_ = true;  
  std::tuple<double, double, double> ret = gainCubature(node->state_, time_of_arrival);
  ROS_DEBUG_STREAM("gain expl: " << std::get<0>(ret));
  return ret;
}

bool AEPlanner::reevaluate(aeplanner::Reevaluate::Request& req,
                           aeplanner::Reevaluate::Response& res)
{

  for (std::vector<geometry_msgs::Point>::iterator it = req.points.begin();
       it != req.points.end(); ++it)
  {
    Eigen::Vector4d pos(it->x, it->y, it->z, 0);

    //Compute the current gain in each cached node
    std::tuple <double, double, double> gain_response = gainCubature(pos, 0);
    ROS_DEBUG_STREAM("gain reeval expl: " << std::get<0>(gain_response));

    //Add the result for each node in a response list
    res.gain.push_back(std::get<0>(gain_response));
    res.dynamic_gain.push_back(std::get<1>(gain_response));
    res.yaw.push_back(std::get<2>(gain_response));
  }
  return true;
}

bool AEPlanner::isPointInGoalFOV(const Eigen::Vector3d& point, const aeplanner::Goal& goal, double fov_angle_rad) {
  // Calculate vector from goal to point
  Eigen::Vector3d goal_pos(goal.x, goal.y, goal.z);
  
  // Create 3D direction vector from goal's yaw
  // Assuming yaw is in the x-y plane, with 0 being along x-axis
  Eigen::Vector3d goal_dir;
  
  double pitch = 0.0;  // Default 
  
  // Full 3D direction with yaw and pitch
  goal_dir = Eigen::Vector3d(
      cos(pitch) * cos(goal.yaw),
      cos(pitch) * sin(goal.yaw),
      sin(pitch)
  );
  
  // Vector from goal to point
  Eigen::Vector3d to_point = point - goal_pos;
  
  // Skip zero vector checks to avoid division by zero
  if (to_point.norm() < 1e-6) {
      return true;  // Point is essentially at the goal position
  }
  
  // Normalize vector to point
  to_point.normalize();
  
  // Calculate angle between goal direction and point
  double dot_product = goal_dir.dot(to_point);
  
  // Clamp to avoid numerical errors
  double angle = acos(std::min(1.0, std::max(-1.0, dot_product)));
  
  // Check if point is within FOV cone
  return angle <= fov_angle_rad / 2.0;
}

/**
 * This function calculates the gain explicitly from the given state (point).
 * It calculates the dynamic gain as well as the static gain and returns them in a tuple
 * with the best yaw angle.
 */
std::tuple <double, double, double> AEPlanner::gainCubature(Eigen::Vector4d state, double time_of_arrival)
{
  visualization_msgs::MarkerArray static_rays;
  visualization_msgs::MarkerArray dynamic_rays;

  // **Declare intenal variables**
  // std::shared_ptr<octomap::OcTree> ot = ot_;

  fast_planner::SDFMap* sdf_map = sdf_map_;

  double static_gain = 0.0;
  double dynamic_gain = 0.0;

  //Field of View
  double fov_y = params_.hfov, fov_p = params_.vfov; //Horizontoal 103.2 deg and Vertical 77.4 deg

  //Radius variables
  double r;
  double r_min = params_.r_min, r_max = params_.r_max;

  //Angles
  int phi, theta;
  double phi_rad, theta_rad;
  
  //Step size for r, phi and theta. In degrees and radians.
  double dr = params_.dr, dphi = params_.dphi, dtheta = params_.dtheta;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;

  //Maps
  std::map<int, double> gain_per_yaw;
  std::map<int, double> dynamic_gain_per_yaw;

  //Set the origin to the point to be examined
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;

  double max_time_step = params_.KFiterations * params_.time_step;

  int index = getCovarianceIndex(max_time_step, params_.time_step, time_of_arrival);
  int ray_id = 0;
  bool blocked = false;

  //For each theta in a circle
  for (theta = -180; theta < 180; theta += dtheta)
  {
    theta_rad = M_PI * theta / 180.0f;
    //For each phi in pitch
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi)
    {
      phi_rad = M_PI * phi / 180.0f;

      //Gain for a specific theta and phi over all r
      double static_g = 0.0;
      double dynamic_g = 0.0;
      visualization_msgs::Marker static_ray;
      visualization_msgs::Marker dynamic_ray;
      blocked = false;

      if(params_.visualize_static_and_dynamic_rays)
      {
        static_ray = createRayMarker(ray_id, "BLUE");
        dynamic_ray = createRayMarker(ray_id, "RED");
      }
      
      //For each r in distance from origin to r_max
      for (r = r_min; r < r_max; r += dr)
      {
        //A point in x,y,z from spherical coordinates
        //So from this volume we are looping over, we extract one point
        vec[0] = origin[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = origin[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = origin[2] + r * cos(phi_rad);

        //Search for the x,y,z point in the OctomapTree
        octomap::point3d query(vec[0], vec[1], vec[2]);
        // octomap::OcTreeNode* result = ot->search(query);

        int occ = sdf_map->getOccupancy(Eigen::Vector3d(vec[0],vec[1],vec[2]));
        // ROS_INFO("Enum occ: %d ", occ);

        //Construct a Vector4d from the point but with yaw = 0
        Eigen::Vector4d v(vec[0], vec[1], vec[2], 0);

        geometry_msgs::Point point;
        point.x = vec[0];
        point.y = vec[1];
        point.z = vec[2];

        if (not blocked and dynamic_mode_ and index != -1)
        {
          blocked = willViewBeBlocked(vec, index, params_.visualize_static_and_dynamic_rays);

        }

        //If the point is outside the map, stop computing gain in that direction
        if (!isInsideBoundaries(v))
        {
          break;
        }
        // if (exploration_map_->queryGaussianValue(Eigen::Vector3d(vec[0],vec[1],vec[2]))>0){
        //   ROS_ERROR_STREAM("Gauss val: "<< exploration_map_->queryGaussianValue(Eigen::Vector3d(vec[0],vec[1],vec[2])));
        // }
        if (!(occ==0) or exploration_map_->isBeingExplored(Eigen::Vector3d(vec[0],vec[1],vec[2]))) //If the point is in known space
        {
          //Iterating forward with r, if the current point is occupied it means 
          //that we have hit a static obstacle
          if (occ == 2)
          {
            //So break the r loop
            break;
          }

          //If the point was free, do not break the loop so continue to loop over r
          //since there might be more unknown space to be found after the known space.

          // or if the point is within other agents fov
        }

        else 
        { 
          double current_gain = (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2); 
          if(pointOnXYBoundaries(v)){
            current_gain *= params_.boost_magnitude;
          }

          // Gaussian view multi-agent check
          // ROS_ERROR_STREAM("Before gain: "<< current_gain);
          // current_gain = exploration_map_->adjustInformationGain(Eigen::Vector3d(vec[0],vec[1],vec[2]),current_gain);
          // ROS_ERROR_STREAM("After gain: "<< current_gain);
               
          if(blocked)
          {
            //The view is blocked by a dynamic obstacle
            //Stop adding gain to the dynamic gain, but continue to add to the static one
            static_g += current_gain;
          }
          else
          {
            //The view is not blocked, add the same gain to static and dynamic
            static_g += current_gain;
            dynamic_g += current_gain;
          }
        }

        if(params_.visualize_static_and_dynamic_rays)
        {
          if(blocked)
          {
            //Visualize static rays behind obstacle
            static_ray.points.push_back(point);
          }
          else
          {
            //Visualize both dynamic and static rays
            static_ray.points.push_back(point);
            dynamic_ray.points.push_back(point);
          }
        }

      }
      gain_per_yaw[theta] += static_g; //Add for each theta what the current gain is (Yaw)
      dynamic_gain_per_yaw[theta] += dynamic_g; //Add for each theta what the current dymamic_gain is.
      
      if(params_.visualize_static_and_dynamic_rays)
      {
        static_rays.markers.push_back(static_ray); //Add ray to rays for visualize
        if(blocked)
        {
          dynamic_rays.markers.push_back(dynamic_ray); //Add ray to rays for visualize  
        }
        else
        {
          dynamic_ray = createRayMarker(ray_id, "RED");
          dynamic_rays.markers.push_back(dynamic_ray);
        }
        ray_id++;
      }
    }
  }

  if(params_.visualize_static_and_dynamic_rays)
  {
    static_rays_marker_pub_.publish(static_rays);
    dynamic_rays_marker_pub_.publish(dynamic_rays);
  }

  //Calculate the best yaw angle
  int best_yaw = 0;
  double best_yaw_score = 0.0, best_dynamic_yaw_score = 0.0;
  
  //For each yaw in one spin
  for (int yaw = -180; yaw < 180; yaw++)
  {
    double yaw_score = 0;
    double dynamic_yaw_score = 0;
    //For each field of view in the y-direction
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++)
    {
      int theta = yaw + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      yaw_score += gain_per_yaw[theta];
      dynamic_yaw_score += dynamic_gain_per_yaw[theta];
    }

    if (best_yaw_score < yaw_score)
    {
      best_yaw_score = yaw_score;
      best_dynamic_yaw_score = dynamic_yaw_score;
      best_yaw = yaw;
    }
  }

  static_gain = best_yaw_score;
  dynamic_gain = best_dynamic_yaw_score;
  double yaw = M_PI * best_yaw / 180.f; //deg2rad
  state[3] = yaw;

  // ROS_INFO("static gain: %f", static_gain);
  return std::make_tuple(static_gain, dynamic_gain, yaw);
}

/**
 * Check if a point will intersect with a dynamic obstacle at time_of_arrival
 * if that is the case, the view will be blocked and we return true. 
*/
bool AEPlanner::willViewBeBlocked(Eigen::Vector3d point, int index, bool visualize_ghosts)
{
  // Race condition
  std::lock_guard<std::mutex> lock(vecMutex);

  if(visualize_ghosts)
  {
    std::vector<std::tuple<double, double, double>> positions;
    for(auto const& person : predicted_data)
    { 
      double x = std::get<0>(person.first)[index];
      double y = std::get<1>(person.first)[index];
      double z = std::get<2>(person.first)[index];
      std::tuple<double, double, double> position = std::make_tuple(x,y,z);
      positions.push_back(position);
    }
    visualizeGhostPedestrian(ghost_marker_pub_, positions);
  }
  
  for(auto const& person : predicted_data)
  { 
    double x = std::get<0>(person.first)[index];
    double y = std::get<1>(person.first)[index];
    double z = std::get<2>(person.first)[index];
    if(isCollisionWithBoundingBox(point, x, y, z))
    {
      return true;
    }
    
  }
  return false;
}

/**
 * Check if a certain point collides with a bounding box, in this case the square
 * of a human obstacle.
*/
bool AEPlanner::isCollisionWithBoundingBox(Eigen::Vector3d point, double x, double y, double z)
{
  return point[0] >= (x - params_.human_width/2) and point[0] <= (x + params_.human_width/2) and
         point[1] >= (y - params_.human_width/2) and point[1] <= (y + params_.human_width/2) and
         point[2] >= z and point[2] <= (z + params_.human_height);
}


geometry_msgs::PoseArray AEPlanner::getFrontiers()
{
  geometry_msgs::PoseArray frontiers;

  pigain::BestNode srv;
  srv.request.threshold = params_.cache_node_threshold; 
  if (best_node_client_.call(srv))
  {
    for (int i = 0; i < srv.response.best_node.size(); ++i)
    {
      geometry_msgs::Pose frontier;
      frontier.position = srv.response.best_node[i];
      frontiers.poses.push_back(frontier);
    }
  }
  return frontiers;
}


bool AEPlanner::isInsideBoundaries(Eigen::Vector4d point)
{  
  return point[0] > params_.boundary_min[0] and point[0] < params_.boundary_max[0] and
         point[1] > params_.boundary_min[1] and point[1] < params_.boundary_max[1] and
         point[2] > params_.boundary_min[2] and point[2] < params_.boundary_max[2];
}

bool AEPlanner::pointOnXYBoundaries(Eigen::Vector4d point)
{ 
  if(point[0] < params_.boundary_min[0] + params_.boosted_boundary_length){
    return true;
  }
  if(point[0] > params_.boundary_max[0] - params_.boosted_boundary_length){
    return true;
  }
  if(point[1] < params_.boundary_min[1] + params_.boosted_boundary_length){
    return true;
  }
  if(point[1] > params_.boundary_max[1] - params_.boosted_boundary_length){
    return true;
  }
  return false;
}


bool AEPlanner::collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r)
{
  ROS_DEBUG_STREAM("Checking collision using SDF map");
  fast_planner::SDFMap* sdf_map  = sdf_map_;

  Eigen::Vector3d start(p1[0], p1[1], p1[2]);
  Eigen::Vector3d end(p2[0], p2[1], p2[2]);
  double r_min = 0.0;
  double r_max = (end - start).norm();
  double dr = sdf_map->getResolution(); // Step size based on SDF resolution

  octomap::point3d start_oct(p1[0], p1[1], p1[2]);
  octomap::point3d end_oct(p2[0], p2[1], p2[2]);
  octomap::point3d min_oct(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r,
                        std::min(p1[2], p2[2]) - r);
  octomap::point3d max_oct(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r,
                        std::max(p1[2], p2[2]) + r);
  double lsq_oct = (end_oct - start_oct).norm_sq();
  double rsq_oct = r * r;

  for (double rl = r_min; rl < r_max; rl += dr)
  {
    Eigen::Vector3d point = start + rl * (end - start).normalized();

    // Check if the point is inside the map
    if (!sdf_map->isInMap(point))
    {
      ROS_WARN_STREAM("Point outside SDF map: " << point.transpose());
    }
    if (sdf_map->getOccupancy(point)==2){
      octomap::point3d pt(point[0], point[1], point[2]);
      if (CylTest_CapsFirst(start_oct, end_oct, lsq_oct, rsq_oct, pt) > 0 or (end - point).norm() < r)
        {
          return true;
        }

    }
    
  }

  ROS_DEBUG_STREAM("No collision detected");
  return false;
}
// {
//   std::shared_ptr<octomap::OcTree> ot = ot_;
//   ROS_DEBUG_STREAM("In collision");
//   octomap::point3d start(p1[0], p1[1], p1[2]);
//   octomap::point3d end(p2[0], p2[1], p2[2]);
//   octomap::point3d min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r,
//                        std::min(p1[2], p2[2]) - r);
//   octomap::point3d max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r,
//                        std::max(p1[2], p2[2]) + r);
//   double lsq = (end - start).norm_sq();
//   double rsq = r * r;

//   for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max),
//                                           it_end = ot->end_leafs_bbx();
//        it != it_end; ++it)
//   {
//     octomap::point3d pt(it.getX(), it.getY(), it.getZ());

//     if (it->getLogOdds() > 0)  // Node is occupied
//     {
//       if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
//       {
//         return true;
//       }
//     }
//   }
//   ROS_DEBUG_STREAM("In collision (exiting)");

//   return false;
// }


void AEPlanner::octomapCallback(const octomap_msgs::Octomap& msg)
{
  ROS_DEBUG_STREAM("Freeing ot_");
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  delete ot;
  ROS_DEBUG_STREAM("Freeing ot_ done:");
}

void AEPlanner::publishEvaluatedNodesRecursive(RRTNode* node)
{
  if (!node)
    return;
  for (typename std::vector<RRTNode*>::iterator node_it = node->children_.begin();
       node_it != node->children_.end(); ++node_it)
  {
    if ((*node_it)->gain_explicitly_calculated_)
    {
      pigain::Node pig_node;
      pig_node.gain = (*node_it)->gain_;
      pig_node.dynamic_gain = (*node_it)->dynamic_gain_;
      pig_node.position.x = (*node_it)->state_[0];
      pig_node.position.y = (*node_it)->state_[1];
      pig_node.position.z = (*node_it)->state_[2];
      pig_node.yaw = (*node_it)->state_[3];
      gain_pub_.publish(pig_node);
    }

    publishEvaluatedNodesRecursive(*node_it);
  }
}


void AEPlanner::agentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_state_[0] = msg.pose.position.x;
  current_state_[1] = msg.pose.position.y;
  current_state_[2] = msg.pose.position.z;
  current_state_[3] = tf2::getYaw(msg.pose.orientation);
  current_state_initialized_ = true;
}


geometry_msgs::Pose AEPlanner::vecToPose(Eigen::Vector4d state)
{
  tf::Vector3 origin(state[0], state[1], state[2]);
  double yaw = state[3];

  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, yaw);
  tf::Pose pose_tf(quat, origin);

  geometry_msgs::Pose pose;
  tf::poseTFToMsg(pose_tf, pose);

  return pose;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James - gjames@NVIDIA.com
// Lisc: Free code - no warranty & no money back.  Use it all you want
// Desc:
//    This function tests if the 3D point 'pt' lies within an arbitrarily
// oriented cylinder.  The cylinder is defined by an axis from 'pt1' to 'pt2',
// the axis having a length squared of 'lsq' (pre-compute for each cylinder
// to avoid repeated work!), and radius squared of 'rsq'.
//    The function tests against the end caps first, which is cheap -> only
// a single dot product to test against the parallel cylinder caps.  If the
// point is within these, more work is done to find the distance of the point
// from the cylinder axis.
//    Fancy Math (TM) makes the whole test possible with only two dot-products
// a subtract, and two multiplies.  For clarity, the 2nd mult is kept as a
// divide.  It might be faster to change this to a mult by also passing in
// 1/lengthsq and using that instead.
//    Elminiate the first 3 subtracts by specifying the cylinder as a base
// point on one end cap and a vector to the other end cap (pass in {dx,dy,dz}
// instead of 'pt2' ).
//
//    The dot product is constant along a plane perpendicular to a vector.
//    The magnitude of the cross product divided by one vector length is
// constant along a cylinder surface defined by the other vector as axis.
//
// Return:  -1.0 if point is outside the cylinder
// Return:  distance squared from cylinder axis if point is inside.
//
//-----------------------------------------------------------------------------
float AEPlanner::CylTest_CapsFirst(const octomap::point3d& pt1,
                                   const octomap::point3d& pt2, float lsq, float rsq,
                                   const octomap::point3d& pt)
{
  float dx, dy, dz;     // vector d  from line segment point 1 to point 2
  float pdx, pdy, pdz;  // vector pd from point 1 to test point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate so pt1 is origin.  Make vector from
  dy = pt2.y() - pt1.y();  // pt1 to pt2.  Need for this is easily eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from pt1 to test point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors to see if point lies behind the
  // cylinder cap at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero the point is behind the pt1 cap.
  // If greater than the cylinder axis line segment length squared
  // then the point is outside the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
    return (-1.0f);
  else
  {
    // Point lies within the parallel caps, so find
    // distance squared from point to line, using the fact that sin^2 + cos^2 = 1
    // the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
    // Carefull: '*' means mult for scalars and dotproduct for vectors
    // In short, where dist is pt distance to cyl axis:
    // dist = sin( pd to d ) * |pd|
    // distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
    // dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
    // dsq = pd * pd - dot * dot / lengthsq
    //  where lengthsq is d*d or |d|^2 that is passed into this function

    // distance squared to the cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
      return (-1.0f);
    else
      return (dsq);  // return distance squared to axis
  }
}

}  // namespace aeplanner
