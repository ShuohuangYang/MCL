//
// Created by bozhou on 12/16/20.
//
#include <ros/ros.h>
#include "apriltag_ros/common_functions.h"
#include <std_msgs/Float64MultiArray.h>

#include "common/homography.h"

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/tf.h>

using namespace std;
using namespace apriltag_ros;


class p2g_tf_node {
public:
    Eigen::Matrix4d sec_in_global_, tag_in_sec_, prim_in_tag_;
    Eigen::Matrix4d prim_in_front, prim_in_left, prim_in_right, prim_in_back;
    bool validData = false;    // seen

    p2g_tf_node() {
      // Hardcode the conversion between the primary and the tags
      prim_in_front <<  0, 0, 1, 0,    1, 0, 0, 0,    0, 1, 0, 0,    0, 0, 0, 1;
      prim_in_left <<  -1, 0, 0, 0,    0, 0, 1, 0,    0, 1, 0, 0,    0, 0, 0, 1;
      prim_in_right <<  1, 0, 0, 0,    0, 0, -1, 0,    0, 1, 0, 0,    0, 0, 0, 1;
      prim_in_back <<  0, 0, -1, 0,    -1, 0, 0, 0,    0, 1, 0, 0,    0, 0, 0, 1;
    }
    ~p2g_tf_node() {}

    void second_to_global_Callback(const nav_msgs::Odometry &msg) {
      // Variables for position
      double x, y, z;
      x = msg.pose.pose.position.x;
      y = msg.pose.pose.position.y;
      z = msg.pose.pose.position.z;

      // Variables for orientation (quaternion)
      tf::Quaternion odom_quat(msg.pose.pose.orientation.x,
                                          msg.pose.pose.orientation.y,
                                          msg.pose.pose.orientation.z,
                                          msg.pose.pose.orientation.w);;

      tf::Matrix3x3 m(odom_quat);

      Eigen::Matrix3d wRo;
      wRo << m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2];
      sec_in_global_.topLeftCorner(3, 3) = wRo;
      sec_in_global_.col(3).head(3) << x, y, z;
      sec_in_global_.row(3) << 0,0,0,1;
    }

    void tag_to_second_Callback(const AprilTagDetectionArray &msg) {
      int cur_id;
      geometry_msgs::PoseStamped pose;
      if (msg.detections.size() > 0)
      {
        validData = true;
      // }
        pose.pose = msg.detections[0].pose.pose.pose;
        cur_id = msg.detections[0].id[0];

        tf::Stamped<tf::Transform> tag_transform;
        tf::poseStampedMsgToTF(pose, tag_transform);

        tf::Transform  transform_tc;
        transform_tc = tag_transform.inverse();

        // Convert tf::Transform to Eigen::Matrix4d
        Eigen::Translation3d tl_btol(transform_tc.getOrigin().getX(),
                                    transform_tc.getOrigin().getY(),
                                    transform_tc.getOrigin().getZ());
        double roll, pitch, yaw;
        tf::Matrix3x3(transform_tc.getRotation()).getEulerYPR(yaw, pitch, roll);

        Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());

        tag_in_sec_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

        // Assign the transformation associated with found tag
        if (cur_id == 0)
          prim_in_tag_ = prim_in_front;
        else if (cur_id == 1)
          prim_in_tag_ = prim_in_back;
        else if (cur_id == 2)
          prim_in_tag_ = prim_in_right;
        else if (cur_id == 3)
          prim_in_tag_ = prim_in_left;
      }
      else{
        validData = false;
      }
    }
};


int main(int argc, char **argv) {
  string robo_id;
  // if (argc >= 2) {
  //   robo_id = argv[1];
  // }
  string robo_odom = robo_id + "/odom";

  ROS_INFO("Publishing the primary's position and orientation in global frame.");

  ros::init(argc, argv, "Primary_To_Global");

  ros::NodeHandle n;

  p2g_tf_node p2g;
  // Subscribe the transformation between the global frame and the secondary frame
  ros::Subscriber Second_in_Global_tf = n.subscribe("/tb3_1/odom",
                                                    1,
                                                    &p2g_tf_node::second_to_global_Callback,
                                                    &p2g);

  // Subscribe the transformation between the secondary frame and the April tag frame
  ros::Subscriber Camera_in_Tag = n.subscribe("/tag_detections",
                                                 1,
                                                 &p2g_tf_node::tag_to_second_Callback,
                                                 &p2g);

  // Publish the primary's position and orientation (4x4 T-matrix) in the global frame
  // Eigen::Matrix4d Primary_on_Global = p2g.sec_in_global_ * p2g.tag_in_sec_ * p2g.prim_in_tag_;
  // std_msgs::Float64MultiArray result;

  // tf::matrixEigenToMsg(Primary_on_Global, result);
  ros::Publisher Primary_in_Global_pub = n.advertise<std_msgs::Float64MultiArray>("Primary_on_Global", 1000);
  // Primary_in_Global_pub.publish(result);

  // ros::spin();
  
  while (ros::ok()) {
    ros::spinOnce();
    if (p2g.validData == true)
    {
      Eigen::Matrix4d Primary_on_Global = p2g.sec_in_global_ * p2g.tag_in_sec_ * p2g.prim_in_tag_;
      std_msgs::Float64MultiArray result;

      tf::matrixEigenToMsg(Primary_on_Global, result);
      Primary_in_Global_pub.publish(result);
    }
    // try
    // {
    //   /* code */
    //   Eigen::Matrix4d Primary_on_Global = p2g.sec_in_global_ * p2g.tag_in_sec_ * p2g.prim_in_tag_;
    //   std_msgs::Float64MultiArray result;

    //   tf::matrixEigenToMsg(Primary_on_Global, result);
    //   Primary_in_Global_pub.publish(result);
    // }
    // catch(...)
    // {
    //   cout << "Exception caught! "<<endl<<endl;
    // }
    // rate.Sleep();
    ROS_INFO("FKING good");
  }
  return 0;
}
