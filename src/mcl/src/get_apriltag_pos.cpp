#include <ros/ros.h>

#include <mcl/StampedAprilTagDetection.h>

#include <apriltag_ros/AprilTagDetection.h>
#include "apriltag_ros/common_functions.h"

// Node that takes the first apriltag seen, extracts the poseStamped msg, 
// and broadcasts it.

class PoseCallback {
  public:
	PoseCallback() {
		pub_ = n_.advertise<mcl::StampedAprilTagDetection>("tag_pose", 10);
		sub_ = n_.subscribe("tag_detections", 10, &PoseCallback::getPoseCallback, this);
	}


	void getPoseCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tags) {
		if (tags->detections.size() <= 0) { 
			return;
		}
		apriltag_ros::AprilTagDetection tag = tags->detections.front();

		mcl::StampedAprilTagDetection sTag;
		sTag.tag = tag;
		sTag.header = tag.pose.header;

		pub_.publish(sTag);
	}

  private:
    ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Publisher idPub_;
	ros::Subscriber sub_;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "extract_apriltag_pose");

	PoseCallback pc;

	ros::spin();
	return 0;
}
