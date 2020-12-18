#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include <deque>
#include <vector>
#include <math.h>
#include <string>

// Retains the lasn `rolling_avg_size_` poses and averages them.

class CombinePoses {
  public:
    CombinePoses() {
        pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/primary/estimated_position", 100);
        sub_ = n_.subscribe<geometry_msgs::Pose>("/primary/poses", 100, &CombinePoses::combine, this);
        
        rolling_avg_size_ = 6;

        debug_ = n_.advertise<std_msgs::String>("/debug", 100);
}

    double yawAngle(const geometry_msgs::Quaternion& q) {
        double q_w = q.w;
        double q_x = q.x;
        double q_y = q.y;
        double q_z = q.z;

        return atan2(2*(q_w*q_z + q_y*q_x), -1.0 + 2 * (q_w*q_w + q_x*q_x));
    }

    void combine(const geometry_msgs::Pose::ConstPtr& pose) {

        if (stored_poses_.size() >= rolling_avg_size_) {
            stored_poses_.pop_back();
        }
        stored_poses_.push_front(*pose);
        int len = stored_poses_.size();

        // I know that we can use the eigen library for a lot of this
        // but I don't want to deal with setting that up.
        std::vector<double> mean;
        mean.push_back(0);
        mean.push_back(0);
        mean.push_back(0);

        for (auto it = stored_poses_.cbegin(); it != stored_poses_.cend(); ++it) {
            mean[0] += it->position.x;
            mean[1] += it->position.y;
            mean[2] += it->position.z;
        }

        mean[0] /= len; mean[1] /= len; mean[2] /= len;

        // "Averaging" the quaternions with Mean of Circular Quantities formula from Wikipedia
        double sinThetaSum = 0; double cosThetaSum = 0;
        double lastSeenTheta = 0;
        for (auto it = stored_poses_.cbegin(); it != stored_poses_.cend(); ++it) {
            double angle = yawAngle(it->orientation);
            
            sinThetaSum += sin(angle);
            cosThetaSum += cos(angle);
            lastSeenTheta = angle;
        }

        double avgAngle = atan2(sinThetaSum, cosThetaSum);

        // Calc covariance vector
        std::vector<double> covariance;
        for (int x = 0; x < 3; x++) {
            for (int y = 0; y < 3; y++) {
                double sum = 0.0;
                std::deque<geometry_msgs::Pose>::iterator it = stored_poses_.begin();
                for (auto it = stored_poses_.cbegin(); it != stored_poses_.end(); ++it){
                    // There has got to be a better way to do this...
                    // ...But I can't think of it right now.

                    std::vector<double> curr;
                    curr.push_back(it->position.x);
                    curr.push_back(it->position.y);
                    curr.push_back(it->position.z);
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
        // O(1) because it's bounded by 36 :)
        boost::array<double, 36> newCovariance;
        for (int i = 0; i < 36; i++) {
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

        geometry_msgs::Quaternion dir;
        dir.w = cos(avgAngle/2);
        dir.x = 0;
        dir.y = 0;
        dir.z = sin(avgAngle/2);

        combinedPose.pose.pose.orientation = dir;

        combinedPose.pose.covariance = newCovariance;

        std_msgs::String endmsg;
        endmsg.data = std::to_string(stored_poses_.size());
        debug_.publish(endmsg);

	    pub_.publish(combinedPose);
    }

  private:
    ros::NodeHandle n_;
	ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Publisher debug_;

    int rolling_avg_size_;

    std::deque<geometry_msgs::Pose> stored_poses_;

}; 

int main(int argc, char** argv){
    ros::init(argc, argv, "combine_poses");

    CombinePoses c;

    ros::spin();
    return 0;
}
