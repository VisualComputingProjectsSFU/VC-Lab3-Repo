/**
 * Created by rakesh on 13/08/18.
 *
 * @file icp_slam_node.cpp
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/mapper.h>
#include <icp_slam/utils.h>

using namespace icp_slam;

/**
 * @brief ROS interface to use ICP-SLAM from icp_slam.h
 */
class ICPSlamNode
{
public:
  ICPSlamNode();

  void laserCallback(const sensor_msgs::LaserScanConstPtr &laser_msg);
  void publishMap(ros::Time timestamp);

protected:

  ros::NodeHandle global_nh_;               // Global namespace ROS handler.
  ros::NodeHandle local_nh_;                // Local namespace ROS handler.
  tf::TransformListener tf_listener_;       // ROS tf (transform) listener.
  tf::TransformBroadcaster tf_broadcaster_; // ROS tf broadcaster.

  ros::Subscriber laser_sub_;               // Laser topic subscriber.

  ros::Publisher map_publisher_;            // Occupancy map publisher.
  ros::Duration map_publish_interval_;      // Time interval to publish map.

  std::string odom_frame_id_;               // Odometry frame ID.
  std::string map_frame_id_;                // Map frame ID.

  boost::shared_ptr<ICPSlam> icp_slam_;

  double x_min_;
  double y_min_;
  double x_max_;
  double y_max_;

  double origin_x_;
  double origin_y_;

  unsigned int width_;                      // @brief Width of the map.
  unsigned int height_;                     // @brief Height of the map.

  double resolution_;
  Mapper mapper_;
  nav_msgs::OccupancyGrid occupancy_grid_;
};

ICPSlamNode::ICPSlamNode() : local_nh_("~")
{
  laser_sub_ = 
    global_nh_.subscribe("base_scan", 10, &ICPSlamNode::laserCallback, this);
  map_publisher_ = local_nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);

  // Getting ROS parameters:
  // local_nh_.param<TYPE>(PARAM_NAME, OUTPUT_VARIABLE, DEFAULT_VALUE)

  double map_update_interval;
  local_nh_.param("map_update_interval", map_update_interval, 5.0);
  map_publish_interval_.fromSec(map_update_interval);

  local_nh_.param<std::string>("odom_frame", odom_frame_id_, "odom");
  local_nh_.param<std::string>("map_frame", map_frame_id_, "map");

  // SLAM parameters.
  double max_keyframes_distance;
  double max_keyframes_angle;
  double max_keyframes_time;
  local_nh_.param<double>(
    "max_keyframes_distance", max_keyframes_distance, 0.02);
  local_nh_.param<double>(
    "max_keyframes_angle", max_keyframes_angle, 0.01);
  local_nh_.param<double>(
    "max_keyframes_time", max_keyframes_time, 0.5);

  local_nh_.param("resolution",  resolution_, 0.05);

  local_nh_.param("xmin", x_min_, -15.0);
  local_nh_.param("ymin", y_min_, -15.0);
  local_nh_.param("xmax", x_max_, 15.0);
  local_nh_.param("ymax", y_max_, 15.0);

  // Initialize the map (width_, height_, origin_).
  width_ = 30.0;
  height_ = 30.0;
  origin_x_ = 15.0;
  origin_y_ = 15.0;

  icp_slam_.reset(
    new ICPSlam(
      max_keyframes_distance, max_keyframes_angle, max_keyframes_time));

  // Occupancy grid.
  occupancy_grid_.data.resize(width_ * height_);
  occupancy_grid_.header.frame_id = map_frame_id_;
  occupancy_grid_.info.width = width_;
  occupancy_grid_.info.height = height_;
  occupancy_grid_.info.resolution = resolution_;
  occupancy_grid_.info.origin.position.x = x_min_;
  occupancy_grid_.info.origin.position.y = y_min_;
  occupancy_grid_.info.origin.orientation.w = 1;
}

void ICPSlamNode::laserCallback(
  const sensor_msgs::LaserScanConstPtr &laser_msg)
{
  static ros::Time last_map_update(0, 0);

  // TODO: Get laser pose in odom frame (using tf).
  tf::StampedTransform tf_odom_laser;

  // Current pose.
  tf::StampedTransform tf_map_laser;
  auto is_keyframe = icp_slam_->track(laser_msg, tf_odom_laser, tf_map_laser);
  if (is_keyframe)
  {
    //TODO: Update the map.
  }

  if (laser_msg->header.stamp - last_map_update > map_publish_interval_)
  {
    publishMap(laser_msg->header.stamp);
  }

  // TODO: Broadcast odom to map transform (using tf).
}

void ICPSlamNode::publishMap(ros::Time timestamp)
{
  // May based on time. 
  // map_publisher_.publish(occupancy_grid_);
}

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "icp_slam_node");
  // ICPSlamNode icp_slam_node;

  // Testing Start.
  icp_slam::ICPSlam slam(NULL, NULL, NULL);

  cv::Mat m1 = cv::Mat(8, 2, CV_32F, cv::Scalar(0, 0));
  cv::Mat m2 = cv::Mat(8, 2, CV_32F, cv::Scalar(0, 0));

  m1.at<float>(0, 0) = 16.8; m1.at<float>(0, 1) = 11.8;
  m1.at<float>(1, 0) = 22.3; m1.at<float>(1, 1) = 7.4;
  m1.at<float>(2, 0) = 30.5; m1.at<float>(2, 1) = 8.8;
  m1.at<float>(3, 0) = 37.0; m1.at<float>(3, 1) = 12.4;
  m1.at<float>(4, 0) = 44.8; m1.at<float>(4, 1) = 12.9;
  m1.at<float>(5, 0) = 51.4; m1.at<float>(5, 1) = 12.3;
  m1.at<float>(6, 0) = 56.2; m1.at<float>(6, 1) = 9.1;
  m1.at<float>(7, 0) = 62.5; m1.at<float>(7, 1) = 6.8;

  m2.at<float>(0, 0) = 3.2; m2.at<float>(0, 1) = 10.8;
  m2.at<float>(1, 0) = 7.1; m2.at<float>(1, 1) = 19.3;
  m2.at<float>(2, 0) = 16.6; m2.at<float>(2, 1) = 20.5;
  m2.at<float>(3, 0) = 23.0; m2.at<float>(3, 1) = 18.1;
  m2.at<float>(4, 0) = 30.8; m2.at<float>(4, 1) = 18.3;
  m2.at<float>(5, 0) = 36.2; m2.at<float>(5, 1) = 22.0;
  m2.at<float>(6, 0) = 44.1; m2.at<float>(6, 1) = 23.1;
  m2.at<float>(7, 0) = 52.0; m2.at<float>(7, 1) = 21.5;

  tf::Transform inv_trans;
  inv_trans.setOrigin(tf::Vector3(1.0, 1.0, 0.0));
  inv_trans.setRotation(tf::Quaternion(0, 0, 0, 1));
  cv::Mat m3 = utils::transformPointMat(inv_trans.inverse(), m1);

  tf::Transform initial_trans;
  initial_trans.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  initial_trans.setRotation(tf::Quaternion(0, 0, 0, 1));

  tf::Transform trans = slam.icpRegistration(m3, m1, initial_trans);
  slam.vizClosestPoints(m1, m3, trans);
  // Testing End.

  // ros::spin();

  return 0;
}
