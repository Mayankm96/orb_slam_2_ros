#ifndef ORB_SLAM_2_INTERFACE
#define ORB_SLAM_2_INTERFACE

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <geometry_msgs/TransformStamped.h>
#include <orb_slam_2/System.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>

#include "orb_slam_2_ros/types.hpp"

namespace orb_slam_2_interface {

// Default values for parameters
static const bool kDefaultVerbose = true;
static const bool kDefaultVisualization = true;
static const std::string kDefaultInterfaceType = "mono";
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultChildFrameId = "cam0";

// Class handling global alignment calculation and publishing
class OrbSlam2Interface {
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor
  OrbSlam2Interface(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);
  // Destructor
  ~OrbSlam2Interface();

  int image_counter = 0;

 protected:
  // Shutdown the interface
  void shutdown();

  // Subscribes and Advertises to the appropriate ROS topics
  void advertiseTopics();
  void getParametersFromRos();

  // Callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Publishing functions
  // Poses
  void publishCurrentPose(const Transformation& T,
                          const std_msgs::Header& header);
  void publishCurrentPoseAsTF(const ros::TimerEvent& event);
  // Trajectory
  void publishTrajectory(const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >& trajectory);
  // Map
  void publishCurrentMap(const std::vector<ORB_SLAM2::MapPoint *> &point_cloud,
                           const sensor_msgs::ImageConstPtr& msg_rgb);
  // Running information
  void publishGBArunning(bool isGBArunning);
  void publishLoopClosing(bool loop_closing);
  void publishEssentialGraphOptimization(bool essential_graph_optimization);

  // Helper functions
  void convertOrbSlamPoseToKindr(const cv::Mat& T_cv, Transformation* T_kindr);

  // Contains a while loop that checks for updates to the past trajectories and then publishes them
  void runPublishUpdatedTrajectory();

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers
  ros::Publisher T_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;
  ros::Publisher Map_pub_;
  ros::Publisher GBA_info_;
  ros::Publisher Loop_closing_info_;
  ros::Publisher Essential_graph_info_;
  ros::Publisher trajectory_pub_;

  // Pointer to the thread that checks for and publishes loop closures
  std::thread* mpt_loop_closure_publisher;

  // The orb slam system
  std::shared_ptr<ORB_SLAM2::System> slam_system_;

  // The current pose
  Transformation T_W_C_;

  // Messages
  sensor_msgs::PointCloud Map_;
  std_msgs::Bool GBA_running_;
  std_msgs::Bool loop_closing_;
  std_msgs::Bool essential_graph_optimization_;

  // Parameters
  bool verbose_;
  std::string vocabulary_file_path_;
  std::string settings_file_path_;

  // Transform frame names
  std::string frame_id_;
  std::string child_frame_id_;

  // Signaling members
  std::mutex m_mutex_shutdown_flag;
  bool mb_shutdown_flag;
};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_INTERFACE */
