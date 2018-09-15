#include "orb_slam_2_ros/interface.hpp"

#include <unistd.h>
#include <thread>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>

#include <glog/logging.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "orb_slam_2_ros/TransformStampedArray.h"
#include "orb_slam_2_ros/interface.hpp"

namespace orb_slam_2_interface {

OrbSlam2Interface::OrbSlam2Interface(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(kDefaultVerbose),
      frame_id_(kDefaultFrameId),
      child_frame_id_(kDefaultChildFrameId),
      mb_shutdown_flag(false)  {
  // Getting data and params
  advertiseTopics();
  getParametersFromRos();
}

OrbSlam2Interface::~OrbSlam2Interface() {
  shutdown();
}

void OrbSlam2Interface::shutdown() {
  // Signaling the shutdown
  std::unique_lock<mutex> lock(m_mutex_shutdown_flag);
  mb_shutdown_flag = true;
}

void OrbSlam2Interface::runPublishUpdatedTrajectory() {
  // Looping while the interface is alive and checking for loop closures
  // TODO(alexmillane): Should be using condition variables really instead of
  // this polled waiting structure.
  while (!mb_shutdown_flag) {
    // Check if updates to the past trajectory are available
    if (slam_system_->isUpdatedTrajectoryAvailable()) {
      // DEBUG
      std::cout << "Updated trajectory available. Publishing." << std::endl;
      // Getting the trajectory from the interface
      std::vector<std::pair<cv::Mat, double>> T_C_W_trajectory_unnormalized =
          slam_system_->GetUpdatedTrajectory();
      // Populating the trajectory message
      orb_slam_2_ros::TransformStampedArray transform_stamped_array_msg;
      for (auto i_T_C_W_stamped_cv = T_C_W_trajectory_unnormalized.begin();
           i_T_C_W_stamped_cv != T_C_W_trajectory_unnormalized.end();
           i_T_C_W_stamped_cv++) {
        // Converting to minkindr
        Transformation T_C_W;
        convertOrbSlamPoseToKindr(i_T_C_W_stamped_cv->first, &T_C_W);
        // Inverting for the proper direction
        Transformation T_W_C = T_C_W.inverse();
        // Converting to a pose message
        geometry_msgs::Pose T_W_C_msg;
        tf::poseKindrToMsg(T_W_C, &T_W_C_msg);
        // Converting to a transform stamped message
        geometry_msgs::TransformStamped T_W_C_msg_2;
        T_W_C_msg_2.header.stamp = ros::Time(i_T_C_W_stamped_cv->second);
        tf::transformKindrToMsg(T_W_C, &T_W_C_msg_2.transform);
        // Pushing this onto the transform stamped array
        transform_stamped_array_msg.transforms.push_back(T_W_C_msg_2);
      }
      // Publishing the trajectory message
      trajectory_pub_.publish(transform_stamped_array_msg);
    }
    usleep(5000);
  }
}

void OrbSlam2Interface::advertiseTopics() {
  // Advertising topics
  T_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
      "transform_cam", 1);
  trajectory_pub_ = nh_private_.advertise<orb_slam_2_ros::TransformStampedArray>(
     "trajectory_cam", 1);
  Map_pub_ = nh_private_.advertise<sensor_msgs::PointCloud>("orbslam_map", 1);
  GBA_info_ = nh_private_.advertise<std_msgs::Bool>("GBA_info", 1);
  Loop_closing_info_ = nh_private_.advertise<std_msgs::Bool>("loop_closing_info",1);
  Essential_graph_info_ = nh_private_.advertise<std_msgs::Bool>("essential_graph_info",1);

  // Creating a callback timer for TF publisher
  tf_timer_ = nh_.createTimer(ros::Duration(0.01),
                              &OrbSlam2Interface::publishCurrentPoseAsTF, this);
}

void OrbSlam2Interface::getParametersFromRos() {
  // Getting the paths to the files required by orb slam
  CHECK(nh_private_.getParam("vocabulary_file_path", vocabulary_file_path_))
      << "Please provide the vocabulary_file_path as a ros param.";
  CHECK(nh_private_.getParam("settings_file_path", settings_file_path_))
      << "Please provide the settings_file_path as a ros param.";
  // Optional params
  nh_private_.getParam("verbose", verbose_);
  nh_private_.getParam("frame_id", frame_id_);
  nh_private_.getParam("child_frame_id", child_frame_id_);
}

void OrbSlam2Interface::publishCurrentPose(const Transformation& T,
                                           const std_msgs::Header& header) {
  // Creating the message
  geometry_msgs::TransformStamped msg;
  // Filling out the header
  // TODO(millane): Should probably just be copying over the time here...
  msg.header = header;
  // Setting the child and parent frames
  msg.child_frame_id = child_frame_id_;
  // Converting from a minkindr transform to a transform message
  tf::transformKindrToMsg(T, &msg.transform);
  // Publishing the current transformation.
  T_pub_.publish(msg);
}

void OrbSlam2Interface::publishCurrentPoseAsTF(const ros::TimerEvent& event) {
  tf::Transform tf_transform;
  tf::transformKindrToTF(T_W_C_, &tf_transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_transform, ros::Time::now(), frame_id_, child_frame_id_));
}

void OrbSlam2Interface::publishTrajectory(const std::vector<Eigen::Affine3d,
                                          Eigen::aligned_allocator<Eigen::Affine3d>>& T_C_W_trajectory) {
  // Populating the pose array
  geometry_msgs::PoseArray pose_array_msg;
  for (size_t pose_index = 0; pose_index < T_C_W_trajectory.size(); pose_index++) {
    Eigen::Affine3d T_C_W = T_C_W_trajectory[pose_index];
    // TODO(alexmillane): This is the wrong place for the inverse. Move it to
    // the extraction function... Also rename the publisher. Gotta go to bed
    // right now.
    Eigen::Affine3d T_W_C = T_C_W.inverse();
    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(T_W_C, pose_msg);
    pose_array_msg.poses.push_back(pose_msg);
  }
  // Publishing
  trajectory_pub_.publish(pose_array_msg);
}

// Publish orbslam2 map to ros point cloud
void OrbSlam2Interface::publishCurrentMap(const std::vector<ORB_SLAM2::MapPoint *>
                                          &point_cloud, const sensor_msgs::ImageConstPtr& msg_rgb) {
  // Creating message
  Map_.points.clear();
  Map_.header.stamp = msg_rgb->header.stamp;
  Map_.header.frame_id = "map";

  for(size_t i = 0; i < point_cloud.size(); i++) {
    if (point_cloud[i]->isBad()){
      continue;
    }
    cv::Mat pos = point_cloud[i]->GetWorldPos();
    geometry_msgs::Point32 pp;
    pp.x = pos.at<float> ( 0 );
    pp.y = pos.at<float> ( 1 );
    pp.z = pos.at<float> ( 2 );

    Map_.points.push_back ( pp );
  }

  Map_pub_.publish(Map_);
}

void OrbSlam2Interface::publishGBArunning(bool isGBArunning){
  GBA_running_.data = isGBArunning;
  GBA_info_.publish(GBA_running_);
}

void OrbSlam2Interface::publishLoopClosing(bool loop_closing) {
  loop_closing_.data = loop_closing;
  Loop_closing_info_.publish(loop_closing_);
}

void OrbSlam2Interface::publishEssentialGraphOptimization(bool essential_graph_optimization) {
  essential_graph_optimization_.data = essential_graph_optimization;
  Essential_graph_info_.publish(essential_graph_optimization_);
}

void OrbSlam2Interface::convertOrbSlamPoseToKindr(const cv::Mat& T_cv,
                                                  Transformation* T_kindr) {
  // Argument checks
  CHECK_NOTNULL(T_kindr);
  CHECK_EQ(T_cv.cols, 4);
  CHECK_EQ(T_cv.rows, 4);
  // Open CV mat to Eigen matrix (float)
  Eigen::Matrix4f T_eigen_f;
  cv::cv2eigen(T_cv, T_eigen_f);
  // Eigen matrix (float) to Eigen matrix (double)
  Eigen::Matrix4d T_eigen_d = T_eigen_f.cast<double>();
  // Extracting and orthonormalizing the rotation matrix
  Eigen::Matrix3d R_unnormalized = T_eigen_d.block<3, 3>(0, 0);
  Eigen::AngleAxisd aa(R_unnormalized);
  Eigen::Matrix3d R = aa.toRotationMatrix();
  // Constructing the transformation
  Quaternion q_kindr(R);
  Eigen::Vector3d t_kindr(T_eigen_d.block<3, 1>(0, 3));
  *T_kindr = Transformation(q_kindr, t_kindr);
}

}  // namespace orb_slam_2_interface
