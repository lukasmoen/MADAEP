#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <regex>

void callback(const nav_msgs::OdometryConstPtr& odom){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y
		, odom->pose.pose.position.z));
	tf::Quaternion q (odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, 
		odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

	static tf::TransformBroadcaster br2;
	tf::Transform transform2;
	transform.setOrigin(tf::Vector3(0.2, 0, 0));
	tf::Quaternion q2;
	q2.setRPY(-1.57, 0, -1.57);
	transform2.setRotation(q2);
	br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "camera_link"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle nh("~");
	std::string model_name;
	if (nh.getParam("name", model_name)) {  // Use "/" for global namespace
		ROS_INFO("Model name retrieved : %s", model_name.c_str());
	} else {
		ROS_WARN("Failed to get param '/model_name'. Using default.");
		model_name = "drone";
	}
	ros::Subscriber sub = nh.subscribe(model_name+"/odom", 10, &callback);
	ros::spin();
	return 0;
}