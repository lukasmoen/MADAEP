#include <fstream>
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <cmath>
#include <aeplanner_evaluation/Coverage.h>
#include <aeplanner_evaluation/Log.h>


// Variables for logging
geometry_msgs::PoseStamped current_pose_stamped;
std::ofstream logfile, pathfile, coveragefile;
bool logging_started = false;
bool exploration_complete = false;
std_msgs::Bool exploration_completed_msg;
bool is_moving = false;
int id = 0;

// Timing variables
double path_length = 0.0;
int iteration = 0;
double elapsed = 0.0;
double total_planning_time = 0.0;
double total_fly_time = 0.0;
double planning_time = 0.0;
double fly_time = 0.0;

ros::Timer timer;
ros::Time starting_time;
ros::Time last_movement_time;
ros::Time last_stationary_time;
aeplanner_evaluation::CoverageResponse srv_response,srv_request;

ros::ServiceClient coverage_srv;
ros::Publisher clock_start_pub, loginfo_pub, collision_pub, completed_pub;


// Calculate distance between two poses and add to the dynamic path length
void add_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    double distance = sqrt((dx * dx) + (dy * dy) + (dz * dz));
    path_length += distance;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (!logging_started) return;
    if (!exploration_complete){
        
    static geometry_msgs::Pose previous_pose = msg->pose;
    
    double distance_moved = sqrt(
        pow(msg->pose.position.x - previous_pose.position.x, 2) +
        pow(msg->pose.position.y - previous_pose.position.y, 2) +
        pow(msg->pose.position.z - previous_pose.position.z, 2));

    // Threshold may need adjustement
    ros::Time current_time = ros::Time::now();

    if (distance_moved > 0.001) {  
        if (!is_moving) {
            // Starting to move
            is_moving = true;
            total_planning_time = planning_time;
        } 
        // Continuously update flying time while moving
        last_movement_time = current_time; 
        fly_time = total_fly_time + (current_time - last_stationary_time).toSec();

    } else {  
        if (is_moving) {
            // Stopped moving
            is_moving = false;
            total_fly_time = fly_time;
        } 
        // Continuously update planning time while stationary
        last_stationary_time = current_time; 
        planning_time = total_planning_time + (current_time - last_movement_time).toSec();
    }

    add_distance(previous_pose, msg->pose);
    previous_pose = msg->pose;

    current_pose_stamped.pose = msg->pose;


    }
    // agent completed exploration, could possibly kill node
    // collision
    std_msgs::Bool msgc;
    msgc.data = true;
    collision_pub.publish(msgc);
}

void timerCallback(const ros::TimerEvent&) {
    if (!logging_started) return;
    if (!exploration_complete){

        elapsed = (ros::Time::now() - starting_time).toSec();

        // PATH
        pathfile << current_pose_stamped.pose.position.x << ", " << current_pose_stamped.pose.position.y << ", " << current_pose_stamped.pose.position.z << ", racer" << std::endl;

        // COVERAGE
        if (coverage_srv.call(srv_request, srv_response)) {
            double coverage_m3 = srv_response.coverage_m3;
            double coverage_p = srv_response.coverage_p;
            double free_space = srv_response.free;
            double occupied_space = srv_response.occupied;
            double unmapped_space = srv_response.unmapped;
            coveragefile << elapsed << ", " << coverage_m3 << ", " << coverage_p << ", " << free_space << ", " << occupied_space << ", " << unmapped_space << std::endl;
        } else {
            ROS_ERROR("5Failed to call service /get_coverage");
        }

        // LOG
        iteration++;


        // Iteration is not accurate.. 
        logfile << iteration << ", "
                << path_length << ", "    
                << elapsed << ", "
                << planning_time << ", "
                << fly_time << std::endl;

        aeplanner_evaluation::Log log_msg;
        log_msg.drone_id = "aeplanner"+std::to_string(id);  
        log_msg.iteration = iteration;
        log_msg.path_length = path_length;
        log_msg.elapsed = elapsed;
        log_msg.total_planning_time = total_planning_time;
        log_msg.total_fly_time = total_fly_time;
        loginfo_pub.publish(log_msg);


        // ROS_INFO_STREAM("Iteration: " << iteration << "  "
        //                 << "Path Length: " << path_length << "  "
        //                 << "Time: " << elapsed << "  "
        //                 << "Planning: " << planning_time << "  "
        //                 << "Flying: " << fly_time);

    }
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
        if (coverage_srv.call(srv_request, srv_response)) 
        {
            // save the coverage, free space, occupied space, and unmapped space in variables in your program
            // Extract the coverage data from the response
            double coverage_m3 = srv_response.coverage_m3;
            double coverage_p = srv_response.coverage_p;
            double free_space = srv_response.free;
            double occupied_space = srv_response.occupied;
            double unmapped_space = srv_response.unmapped;
            coveragefile << 0 << ", " << 0 << ", " <<  0 << ", " << 0 << ", "  << 0 << ", " <<  unmapped_space+occupied_space+free_space << std::endl;
        } 
        else
        {
            ROS_ERROR("6Failed to call service /get_coverage");
        }
    }
}

void completedCallback(const std_msgs::Bool::ConstPtr& msg) {
    // Exploration is complete
    if (msg->data){
        ROS_INFO("[Logger] Exploration completed.");
        exploration_complete = true;

        std_msgs::String exploration_completed_msg;
        exploration_completed_msg.data = "/aeplanner"+std::to_string(id)+",true";
        completed_pub.publish(exploration_completed_msg);

        // You can also take additional actions like safely shutting down the node
        
    }
    // ros::shutdown();
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "logger_node");
    ros::NodeHandle nh("~");  // Private namespace
    ROS_INFO("Logger node started");

    // Fetch ID from the private namespace
    if (!nh.getParam("id", id)) {
        ROS_ERROR("Failed to get 'id' parameter. Defaulting to 0.");
        id = 0;  // Default ID if not set
    }

    // Open log files with aeplanner ID
    std::string homePath = std::getenv("HOME");
    std::string logPath = homePath + "/data/aeplanner" + std::to_string(id);
    
    logfile.open(logPath + "/logfile.csv");
    pathfile.open(logPath + "/path.csv");
    coveragefile.open(logPath + "/coverage.csv");

    // File Headers
    logfile << "Iteration, Path length, Time, Planning, Flying" << std::endl;
    pathfile << "Goal x, Goal y, Goal z, Planner" << std::endl;
    coveragefile << "Time, Coverage (m3), Coverage (%), Free space, Occupied Space, Unmapped Space" << std::endl;

    // ROS Subscribers
    ros::Subscriber sub_pose = nh.subscribe("/drone" + std::to_string(id) + "/pose", 10, poseCallback);
    ros::Subscriber sub_start_logging = nh.subscribe("/drone" + std::to_string(id) + "/start_logging", 1, startLoggingCallback);
    ros::Subscriber exploration_completion_sub = nh.subscribe("/drone" + std::to_string(id) + "/exploration_completed", 1, completedCallback);

    // for stopping sim (run_experiments)
    completed_pub = nh.advertise<std_msgs::String>("/exploration_completed", 1);

    // ROS Publishers
    collision_pub = nh.advertise<std_msgs::Bool>("/drone" + std::to_string(id) + "/write_log", 1);
    clock_start_pub = nh.advertise<std_msgs::Bool>("/drone" + std::to_string(id) + "/clock_start", 1);
    loginfo_pub = nh.advertise<aeplanner_evaluation::Log>("/loginfo", 1);
   
    // ROS Services
    coverage_srv = nh.serviceClient<aeplanner_evaluation::Coverage>("/drone" + std::to_string(id) + "/get_coverage");

    // timer 
    timer = nh.createTimer(ros::Duration(1), timerCallback);

    ros::spin();

    // Close log files
    logfile.close();
    pathfile.close();
    coveragefile.close();

    return 0;
}

