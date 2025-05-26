#include <fstream>
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <cmath>
#include <aeplanner_evaluation/Coverage.h>
#include <aeplanner_evaluation/Log.h>

// Variables for logging
geometry_msgs::PoseStamped current_pose_stamped;
std::ofstream logfile, pathfile, coveragefile, intervalsfile;
bool logging_started = false;
bool exploration_complete = false;
std_msgs::Bool exploration_completed_msg;
bool is_moving = false;
struct PlannerData {
    double total_path_length = 0.0;
    double total_planning_time = 0.0;
    double total_fly_time = 0.0;
    double latest_elapsed = 0.0;
};
std::map<std::string, PlannerData> planner_logs;
std::map<std::string, double> drone_coverages;

// Timing variables
double path_length = 0.0;
int iteration = 0;
double elapsed = 0.0;
double total_planning_time = 0.0;
double total_fly_time = 0.0;
double planning_time = 0.0;
double fly_time = 0.0;

int drone_num;

ros::Time starting_time;
ros::Time last_movement_time;
ros::Time last_stationary_time;
aeplanner_evaluation::Coverage srv_response, srv_request;

ros::ServiceClient coverage_srv;
ros::Publisher collision_pub;
ros::Publisher clock_start_pub;

// Calculate distance between two poses and add to the dynamic path length
void add_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    double distance = sqrt((dx * dx) + (dy * dy) + (dz * dz));
    path_length += distance;
}

void startLoggingCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data && !logging_started) {
        logging_started = true;
        ROS_INFO("Logging started");
        last_movement_time = ros::Time::now();
        last_stationary_time = ros::Time::now();
        starting_time = ros::Time::now();
        std_msgs::Bool planning_started;
        planning_started.data = true;
        clock_start_pub.publish(planning_started);
    }
}

void completedCallback(const std_msgs::Bool::ConstPtr& msg) {
    // Exploration is complete
    if (msg->data){
        ROS_INFO("[Merged logger] Exploration completed.");
    }
}

void logCallback(const aeplanner_evaluation::Log::ConstPtr& msg){
    // collision
    // ROS_INFO("Log callback");
    std_msgs::Bool msgc;
    msgc.data = true;
    collision_pub.publish(msgc);

    std::string drone_id = msg->drone_id;
    planner_logs[drone_id].total_path_length = msg->path_length;
    planner_logs[drone_id].total_planning_time = msg->total_planning_time;
    planner_logs[drone_id].total_fly_time = msg->total_fly_time;
    planner_logs[drone_id].latest_elapsed = msg->elapsed;

    // Compute total values
    double total_path_length = 0.0;
    double total_planning_time = 0.0;
    double total_fly_time = 0.0;
    double latest_elapsed = 0.0;
    
    for (const auto& entry : planner_logs) {
        total_path_length += entry.second.total_path_length;
        total_planning_time += entry.second.total_planning_time;
        total_fly_time += entry.second.total_fly_time;
        if (entry.second.latest_elapsed > latest_elapsed) {
            latest_elapsed = entry.second.latest_elapsed;
        }
    }

    logfile << iteration << ", "
            << total_path_length << ", "    
            << latest_elapsed << ", "
            << total_planning_time << ", "
            << total_fly_time << std::endl;
    iteration++;

    // COVERAGE
    aeplanner_evaluation::Coverage srv;
    if (coverage_srv.call(srv)) {
        // ROS_INFO("Coverage Response: coverage_m3 = %f, coverage_p = %f, free = %f, occupied = %f, unmapped = %f",
        //          srv.response.coverage_m3, srv.response.coverage_p, srv.response.free, srv.response.occupied, srv.response.unmapped);
        
        coveragefile << latest_elapsed << ", "
                     << srv.response.coverage_m3 << ", "
                     << srv.response.coverage_p << ", "
                     << srv.response.free << ", "
                     << srv.response.occupied << ", "
                     << srv.response.unmapped << std::endl;
    } else {
        ROS_ERROR("[LOG_MERG] Failed to call service /get_coverage");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "logger_node");
    ros::NodeHandle nh("~");  // Private namespace
    ROS_INFO("Logger node started");

    // Fetch ID from the private namespace
    int id;
    if (!nh.getParam("id", id)) {
        ROS_ERROR("Failed to get 'id' parameter. Defaulting to 0.");
        id = 0;  // Default ID if not set
    }
    
    if (!nh.getParam("drone_num", drone_num)) {
        ROS_ERROR("Failed to get 'drone_num' parameter. Defaulting to 0.");
        drone_num = 0;  // Default ID if not set
    }
    // Open log files with aeplanner ID
    std::string homePath = std::getenv("HOME");
    std::string logPath = homePath + "/data/aeplanner0";
    
    logfile.open(logPath + "/logfile.csv");
    pathfile.open(logPath + "/path.csv");
    coveragefile.open(logPath + "/coverage.csv");
    intervalsfile.open(logPath + "/intervals.csv");

    // File Headers
    logfile << "Iteration, Path length, Time, Planning, Flying" << std::endl;
    pathfile << "Goal x, Goal y, Goal z, Planner" << std::endl;
    coveragefile << "Time, Coverage (m3), Coverage (%), Free space, Occupied Space, Unmapped Space" << std::endl;
    intervalsfile << "start-time,duration,end-time"<< std::endl;

    // ROS Subscribers
    ros::Subscriber sub_start_logging = nh.subscribe("/drone" + std::to_string(id) + "/start_logging", 1, startLoggingCallback);
    ros::Subscriber exploration_completion_sub = nh.subscribe("/drone" + std::to_string(id) + "/exploration_completed", 1, completedCallback);
    ros::Subscriber log_sub = nh.subscribe("/loginfo", 1, logCallback);

    // ROS Publishers
    collision_pub = nh.advertise<std_msgs::Bool>("/drone" + std::to_string(id) + "/write_log", 1);
    clock_start_pub = nh.advertise<std_msgs::Bool>("/drone" + std::to_string(id) + "/clock_start", 1);

    // ROS Services
    coverage_srv = nh.serviceClient<aeplanner_evaluation::Coverage>("/get_coverage0");

    ros::spin();

    // Close log files
    logfile.close();
    pathfile.close();
    coveragefile.close();
    intervalsfile.close();
    return 0;
}
