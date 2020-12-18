#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/console.h>

#include <mcl/StampedAprilTagDetection.h>
#include "apriltag_ros/common_functions.h"

#include <vector>
#include <math.h>
#include <cmath>
#include <string>

class SecondaryToPrimary {
  public:
    SecondaryToPrimary() {
        pub_ = n_.advertise<geometry_msgs::Pose>("/primary/poses",100);
        sub_tpose_.subscribe(n_, "tag_pose", 1);
        sub_cpose_.subscribe(n_, "amcl_pose", 1);
        
        sync_.reset(new Sync(MySyncPolicy(5), sub_tpose_, sub_cpose_));
        sync_->registerCallback(boost::bind(&SecondaryToPrimary::transformPointCallback, this, _1, _2));

        debug_ = n_.advertise<std_msgs::String>("/debug", 100);
    }

    void transformPointCallback(
      const mcl::StampedAprilTagDetection::ConstPtr& tagWrapper,
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& cpose) {

        apriltag_ros::AprilTagDetection tag = tagWrapper->tag;

        geometry_msgs::PoseWithCovarianceStamped tpose = tag.pose;

	    geometry_msgs::Point rel_pose = tpose.pose.pose.position;
        double rel_x = rel_pose.x;
        double rel_z = rel_pose.z;

        geometry_msgs::Quaternion tpose_quat = tpose.pose.pose.orientation;
        double tq_x = tpose_quat.x;
        double tq_y = tpose_quat.y;
        double tq_z = tpose_quat.z;
        double tq_w = tpose_quat.w;

        // Formula for yaw around y axis (relative to QR code)
        double theta_april = asin(2*(tq_w * tq_y - tq_z * tq_x));

        // Which tag are we looking at? We handle each of them separately!
        // If this tag has no id, then... not worrying about that right now.
        int id_april = tag.id.front();

        geometry_msgs::Point cpose_pos = cpose->pose.pose.position;
        double cpose_x = cpose_pos.x;
        double cpose_y = cpose_pos.y;

    	geometry_msgs::Quaternion cpose_quat = cpose->pose.pose.orientation;
        double cq_x = cpose_quat.x;
        double cq_y = cpose_quat.y;
        double cq_z = cpose_quat.z;
        double cq_w = cpose_quat.w;

        // Formula for yaw. Shamelessly stolen from stack overflow
        double theta = atan2(2*(cq_w*cq_z + cq_y*cq_x), -1.0 + 2 * (cq_w*cq_w + cq_x*cq_x));

        geometry_msgs::Point newpt;
        newpt.x = cpose_x + rel_z * cos(theta) + rel_x * sin(theta);
        newpt.y = cpose_y + rel_z * sin(theta) + rel_x * cos(theta);
        newpt.z = 0.1; // Doesn't matter. We're not flying

        double theta_new = theta + theta_april;

        // Need to transform different sides by a different angle
        switch(id_april) {
            case 0:
                theta_new += 1.571;
                break;
            case 2:
                theta_new += 4.712;
                break;
            case 3:
                theta_new += 0.785;
                break;
            // case 1: theta_new += 0;
        }
        theta_new = std::fmod(theta_new, 3.1415926);

        geometry_msgs::Quaternion newrot;
        newrot.w = cos(theta_new/2);
        newrot.x = 0;
        newrot.y = 0;
        newrot.z = sin(theta_new/2);

        geometry_msgs::Pose thePose;
        thePose.position = newpt;
        thePose.orientation = newrot;

        std_msgs::String dmsg;
        dmsg.data = std::to_string(theta);
        debug_.publish(dmsg);

	    pub_.publish(thePose);
    }

  private:
    ros::NodeHandle n_;
	ros::Publisher pub_;
    ros::Publisher debug_;
	message_filters::Subscriber<mcl::StampedAprilTagDetection> sub_tpose_;
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_cpose_;
    typedef message_filters::sync_policies::ApproximateTime<
        mcl::StampedAprilTagDetection, 
        geometry_msgs::PoseWithCovarianceStamped
    > MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
}; // End of class SecondaryToPrimary

int main(int argc, char** argv){
    ros::init(argc, argv, "calc_detected_primary_pos");

    SecondaryToPrimary stop;

    ros::spin();
    return 0;
}
