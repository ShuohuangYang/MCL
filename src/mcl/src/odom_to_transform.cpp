#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

// Simple node that takes Odometry messages from Gazebo
// and then turns them into tf messages from `odom` to `base_footprint`
// lol also turns odom into base_footprint via identity

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	static tf::TransformBroadcaster br;
	
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(
				msg->pose.pose.position.x,
				msg->pose.pose.position.y,
				msg->pose.pose.position.z));
	
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
	transform.setRotation(q);

	// tf::Transform t_id;
	// t_id.setIdentity();
	// br.sendTransform(tf::StampedTransform(t_id, ros::Time::now(), "odom", "base_footprint"));
	
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "map_to_odom_tf_broadcaster");
	
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("/odom", 10, &odomCallback);

	ros::spin();
	return 0;
}
