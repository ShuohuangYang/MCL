#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/console.h>

#include <vector>
#include <math.h>
#include <string>

class SecondaryToPrimary {
  public:
    SecondaryToPrimary() {
        pub_ = n_.advertise<geometry_msgs::Point>("/primary/points",100);
        sub_tpose_.subscribe(n_, "tag_pose", 1);
        sub_cpose_.subscribe(n_, "amcl_pose", 1);
        
        sync_.reset(new Sync(MySyncPolicy(5), sub_tpose_, sub_cpose_));
        sync_->registerCallback(boost::bind(&SecondaryToPrimary::transformPointCallback, this, _1, _2));

        debug_ = n_.advertise<std_msgs::String>("/debug", 100);
    }

    void transformPointCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& tpose,
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& cpose) {

	    geometry_msgs::Point rel_pose = tpose->pose.pose.position;
        double rel_x = rel_pose.x;
        double rel_z = rel_pose.z;

        geometry_msgs::Point cpose_pos = cpose->pose.pose.position;
        double cpose_x = cpose_pos.x;
        double cpose_y = cpose_pos.y;

    	geometry_msgs::Quaternion cpose_quat = cpose->pose.pose.orientation;
        double q_x = cpose_quat.x;
        double q_y = cpose_quat.y;
        double q_z = cpose_quat.z;
        double q_w = cpose_quat.w;

        // Formula for yaw. Shamelessly stolen from stack overflow
        double theta = atan2(2*(q_w*q_z + q_y*q_x), -1.0 + 2 * (q_w*q_w + q_x*q_x));

        geometry_msgs::Point newpt;
        newpt.x = cpose_x + rel_z * cos(theta) + rel_x * sin(theta);
        newpt.y = cpose_y + rel_z * sin(theta) + rel_x * cos(theta);
        newpt.z = 0.1; // Doesn't matter. We're not flying

        std_msgs::String dmsg;
        dmsg.data = std::to_string(theta);
        debug_.publish(dmsg);

	    pub_.publish(newpt);
    }

  private:
    ros::NodeHandle n_;
	ros::Publisher pub_;
    ros::Publisher debug_;
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_tpose_;
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_cpose_;
    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::PoseWithCovarianceStamped, 
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
