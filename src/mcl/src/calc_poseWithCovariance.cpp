#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include <deque>
#include <vector>
#include <math.h>
#include <string>

class PointsToCovariance {
  public:
    PointsToCovariance() {
        pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/primary/estimated_position", 100);
        sub_ = n_.subscribe<geometry_msgs::Point>("/primary/points", 100, &PointsToCovariance::covarianceCallback, this);
        
        rolling_avg_size_ = 6;

        debug_ = n_.advertise<std_msgs::String>("/debug", 100);
    }

    void covarianceCallback(const geometry_msgs::Point::ConstPtr& point) {

        std_msgs::String dmsg;
        dmsg.data = "START";
        debug_.publish(dmsg);

        if (stored_points_.size() >= rolling_avg_size_) {
            stored_points_.pop_back();
        }
        stored_points_.push_front(*point);
        int len = stored_points_.size();

        // I know that we can use the eigen library for a lot of this
        // but I don't want to deal with setting that up.
        std::vector<double> mean;
        mean.push_back(0);
        mean.push_back(0);
        mean.push_back(0);

        for (auto it = stored_points_.cbegin(); it != stored_points_.cend(); ++it) {
            mean[0] += it->x;
            mean[1] += it->y;
            mean[2] += it->z;
        }

        mean[0] /= len; mean[1] /= len; mean[2] /= len;

        // Calc covariance vector
        std::vector<double> covariance;
        for (int x = 0; x < 3; x++) {
            for (int y = 0; y < 3; y++) {
                double sum = 0.0;
                std::deque<geometry_msgs::Point>::iterator it = stored_points_.begin();
                for (auto it = stored_points_.cbegin(); it != stored_points_.end(); ++it){
                    // There has got to be a better way to do this...
                    // ...But I can't think of it right now.

                    std::vector<double> curr;
                    curr.push_back(it->x);
                    curr.push_back(it->y);
                    curr.push_back(it->z);
                    sum += (curr[x] - mean[x])*(curr[y] - mean[y]);
                }
                sum /= (len - 1);
                covariance.push_back(sum);
            }
            // Padding, because no orientation covariance (yet)
            for (int i = 0; i < 3; i++) {
                covariance.push_back(0.0);
            }
        }

        // Padding for the last three rows, for the same reason
        for (int i = 0; i < 6; i++) {
            covariance.push_back(0.0);
        }

        // Getting the right datatype.
        // O(1) option because it's bounded by 36 :)
        boost::array<double, 36> newCovariance;
        for (int i = 0; i < 36; i++) {
            dmsg.data = "covloop";
            debug_.publish(dmsg);
            newCovariance[i] = covariance[i];
        }

        // Create Message to be sent
        geometry_msgs::PoseWithCovarianceStamped combinedPose;
        combinedPose.header.stamp = ros::Time::now();
        combinedPose.header.frame_id = "/map"; // SHOULD BE WORLD_FRAME

        geometry_msgs::Point thePoint;
        thePoint.x = mean[0];
        thePoint.y = mean[1];
        thePoint.z = mean[2];
        combinedPose.pose.pose.position = thePoint;

        // Constant for now. Will set dynamically soon. With finite differencing?
        geometry_msgs::Quaternion dir;
        dir.x = 0;
        dir.y = 0;
        dir.z = 1;
        dir.w = 0;
        combinedPose.pose.pose.orientation = dir;

        combinedPose.pose.covariance = newCovariance;

        std_msgs::String endmsg;
        endmsg.data = std::to_string(stored_points_.size());
        debug_.publish(endmsg);

	    pub_.publish(combinedPose);
    }

  private:
    ros::NodeHandle n_;
	ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Publisher debug_;

    int rolling_avg_size_;

    std::deque<geometry_msgs::Point> stored_points_;

}; 

int main(int argc, char** argv){
    ros::init(argc, argv, "calc_poseWithCovariance_from_points");

    PointsToCovariance pointsToPoseWithCovariance;

    ros::spin();
    return 0;
}
