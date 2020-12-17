#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>

#include <deque>
#include <vector>
#include <math.h>
#include <string>

class PointsToCovariance {
  public:
    PointsToCovariance() {
        pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/primary/estimated_position", 100);
        sub_ = n_.subscribe<geometry_msgs::Point>("/primary/points", 100, &PointsToCovariance::covarianceCallback, this);
        
        rolling_avg_size_ = 5;

        debug_ = n_.advertise<std_msgs::String>("/debug", 100);
    }

    void covarianceCallback(const geometry_msgs::Point::ConstPtr& point) {

        if (stored_points_.size() >= rolling_avg_size_) {
            stored_points_.pop_back();
        }
        stored_points_.push_front(&point);
        int len = stored_points_.size();

        // I know that we can use the eigen library for a lot of this
        // but I don't want to deal with setting that up.
        std::vector<double> mean;
        mean.push_back(0);
        mean.push_back(0);
        mean.push_back(0);

        // Calc the mean vector
        std::deque<geometry_msgs::Point>::iterator it = stored_points_.begin();
        while (it != stored_points_.end()) {
            mean[0] += it.x;
            mean[1] += it.y;
            mean[2] += it.z;
        }
        x_avg /= len; y_avg /= len; z_avg /= len;

        // Calc covariance vector
        std::vector<double> covariance;
        for (x = 0; x < 3; x++) {
            for (y = 0; y < 3; y++) {
                double sum = 0.0;
                std::deque<geometry_msgs::Point>::iterator it = stored_points_.begin();
                while (it != stored_points_.end()){
                    // There has got to be a better way to do this...
                    // ...But I can't think of it right now.

                    std::vector<double> curr;
                    curr.push_back(it.x);
                    curr.push_back(it.y);
                    curr.push_back(it.z);
                    sum += (curr[x] - mean[x])*(curr[y] - mean[y]);
                }
                sum /= (len - 1);
                covariance.push_back(sum);
            }
            // Padding, because no orientation covariance (yet)
            for (i = 0; i < 3; i++) {
                covariance.push_back(0.0);
            }
        }

        // Padding for the last three rows, for the same reason
        for (i = 0; i < 6; i ++) {
            covariance.push_back(0.0);
        }


        std_msgs::String dmsg;
        dmsg.data = std::to_string(theta);
        debug_.publish(dmsg);

	    pub_.publish(newpt);
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
    ros::init(argc, argv, "calc_detected_primary_pos");

    SecondaryToPrimary stop;

    ros::spin();
    return 0;
}
