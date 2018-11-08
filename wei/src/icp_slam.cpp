/**
 * Created by rakesh on 13/08/18.
 *
 * @file icp_slam.cpp
 */

#include <cmath>
#include <map>
#include <numeric>
#include <chrono>

#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>

#define TIME_DIFF(tic, toc) \
  ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{

  ICPSlam::ICPSlam(
    tfScalar max_keyframes_distance, 
    tfScalar max_keyframes_angle, 
    double max_keyframes_time): 
      max_keyframes_distance_(max_keyframes_distance),
      max_keyframes_angle_(max_keyframes_angle),
      max_keyframes_time_(max_keyframes_time),
      last_kf_laser_scan_(new sensor_msgs::LaserScan()),
      is_tracker_running_(false)
  {
    last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
  }

  bool ICPSlam::track(
    const sensor_msgs::LaserScanConstPtr &laser_scan,
    const tf::StampedTransform &current_frame_tf_odom_laser,
    tf::StampedTransform &tf_map_laser)
  {
    if (is_tracker_running_)
    {
      ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running.");
      return false;
    }

    // TODO: find the pose of laser in map frame.
    // If a new keyframe is created, run ICP.
    // If not a keyframe, obtain the laser pose in map frame based on odometry
    // update.
  }

  bool ICPSlam::isCreateKeyframe(
    const tf::StampedTransform &current_frame_tf, 
    const tf::StampedTransform &last_keyframe_tf) const
  {
    assert(
      current_frame_tf.frame_id_ == last_keyframe_tf.frame_id_);
    assert(
      current_frame_tf.child_frame_id_ == last_keyframe_tf.child_frame_id_);

    // TODO: Check whether you want to create keyframe (based on 
    // max_keyframes_distance_, max_keyframes_angle_, max_keyframes_time_).

    return true;
  }

  void ICPSlam::closestPoints(
    cv::Mat &point_mat1,
    cv::Mat &point_mat2,
    std::vector<int> &closest_indices,
    std::vector<float> &closest_distances_2)
  {
    // Uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/
    // iterative-closest-point-icp-with-opencv-w-code/
    closest_indices = std::vector<int>(point_mat1.rows, -1);
    closest_distances_2 = std::vector<float>(point_mat1.rows, -1);


    cv::Mat multi_channeled_mat1;
    cv::Mat multi_channeled_mat2;

    point_mat1.convertTo(multi_channeled_mat1, CV_32FC2);
    point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

    cv::flann::Index flann_index(
      multi_channeled_mat2,
      cv::flann::KDTreeIndexParams(2));  // Using 2 randomized kdtrees.

    cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
    cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
    flann_index.knnSearch(
      multi_channeled_mat1, 
      mat_indices, mat_dists, 
      1, 
      cv::flann::SearchParams(64));

    int* indices_ptr = mat_indices.ptr<int>(0);
    for (int i = 0; i < mat_indices.rows; ++i) {
      closest_indices[i] = indices_ptr[i];
    }

    mat_dists.copyTo(cv::Mat(closest_distances_2));

    // --------------------------- Naive Version --------------------------- //
    // Max allowed distance between corresponding points
    // const float max_distance = 0.5;
    //
    // for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
    // {
    //   int closest_point_idx = -1;
    //   float closest_distance_2 = std::pow(max_distance, 2.0f);
    //
    //   for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
    //   {
    //     auto distance2 =
    //       std::pow(
    //         point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f) + 
    //       std::pow(
    //         point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
    //
    //     if (distance2 < closest_distance_2)
    //     {
    //       closest_distance_2 = distance2;
    //       closest_point_idx = (int)j;
    //     }
    //   }
    //
    //   if (closest_point_idx >= 0)
    //   {
    //     closest_indices[i] = closest_point_idx;
    //     closest_distances_2[i] = closest_distance_2;
    //   }
    // }
  }

  void ICPSlam::vizClosestPoints(
    cv::Mat &original_point_mat1,
    cv::Mat &original_point_mat2,
    const tf::Transform &trans)
  {
    assert(original_point_mat1.size == original_point_mat2.size);

    // Transform the second matrix. One more dimension is added!
    cv::Mat point_mat2 = 
      utils::transformPointMat(trans.inverse(), original_point_mat2);
    cv::Mat point_mat1 = original_point_mat1;

    const float resolution = 0.005;

    // Compute canvas size.
    float min_x = point_mat1.at<float>(0, 0);
    float min_y = point_mat1.at<float>(0, 1);
    float max_x = point_mat1.at<float>(0, 0);
    float max_y = point_mat1.at<float>(0, 1);

    for (int i = 0; i < (size_t)point_mat1.rows; i++)
    {
      min_x = 
        point_mat1.at<float>(i, 0) < min_x ? 
        point_mat1.at<float>(i, 0) : min_x;
      min_y = 
        point_mat1.at<float>(i, 1) < min_y ? 
        point_mat1.at<float>(i, 1) : min_y;
      max_x = 
        point_mat1.at<float>(i, 0) > max_x ? 
        point_mat1.at<float>(i, 0) : max_x;
      max_y = 
        point_mat1.at<float>(i, 1) > max_y ? 
        point_mat1.at<float>(i, 1) : max_y;

      min_x = 
        point_mat2.at<float>(i, 0) < min_x ? 
        point_mat2.at<float>(i, 0) : min_x;
      min_y = 
        point_mat2.at<float>(i, 1) < min_y ? 
        point_mat2.at<float>(i, 1) : min_y;
      max_x = 
        point_mat2.at<float>(i, 0) > max_x ? 
        point_mat2.at<float>(i, 0) : max_x;
      max_y = 
        point_mat2.at<float>(i, 1) > max_y ? 
        point_mat2.at<float>(i, 1) : max_y;
    }

    // Create the image.
    int img_size = 1024;
    float padding = 0.1;
    cv::Mat img(
      (int)(img_size * (1 - padding)), 
      (int)(img_size * (1 - padding)), 
      CV_8UC3, cv::Scalar(0, 0, 0));

    // Translate both matrices.
    float content_size = 0;
    float resize_ratio = 0;
    float x_translate = 0;
    float y_translate = 0;
    tf::Transform t;
    if (max_x - min_x > max_y - min_y)
    {
      content_size = max_x - min_x;
      x_translate = min_x;
      y_translate = min_y + (max_y - min_y + content_size) / 2.0;

    } 
    else 
    {
      content_size = max_y - min_y;
      x_translate = min_x + (max_x - min_x - content_size) / 2.0;
      y_translate = max_y;
    }

    t.setOrigin(tf::Vector3(x_translate, y_translate, 0.0));
    t.setRotation(tf::Quaternion(0, 0, 0, 1));
    resize_ratio = img_size * (1 - padding) / content_size;
    cv::circle(img, 
      cv::Point(-x_translate * resize_ratio, y_translate * resize_ratio), 
      5, cv::Scalar(0, 255, 0), -1);
    cv::line(img, 
      cv::Point(-x_translate * resize_ratio - 10, y_translate * resize_ratio), 
      cv::Point(-x_translate * resize_ratio + 10, y_translate * resize_ratio), 
      cv::Scalar(0, 255, 0), 2);
    cv::line(img, 
      cv::Point(-x_translate * resize_ratio, y_translate * resize_ratio - 10), 
      cv::Point(-x_translate * resize_ratio, y_translate * resize_ratio + 10), 
      cv::Scalar(0, 255, 0), 2);

    // Scale both matrices.
    point_mat1 = utils::transformPointMat(t.inverse(), point_mat1);
    point_mat2 = utils::transformPointMat(t.inverse(), point_mat2);
    point_mat1 = point_mat1.mul(resize_ratio);
    point_mat2 = point_mat2.mul(resize_ratio);

    // Plot points from both matrices.
    for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
    {
      float x1 = point_mat1.at<float>(i, 0);
      float y1 = -point_mat1.at<float>(i, 1);
      float x2 = point_mat2.at<float>(i, 0);
      float y2 = -point_mat2.at<float>(i, 1);

      cv::Point point1(x1, y1);
      cv::Point point2(x2, y2);

      cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
      cv::circle(img, point2, 5, cv::Scalar(255, 0, 0), -1);

      cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2);
    }

    // Add padding and save results.
    cv::Mat img_pad(img_size, img_size, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::copyMakeBorder(
      img, img_pad, 
      (int)(img_size * padding), 
      (int)(img_size * padding), 
      (int)(img_size * padding), 
      (int)(img_size * padding), 
      cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::Mat tmp;
    cv::flip(img_pad, tmp, 0);
    cv::imwrite("/tmp/viz_closest_points.png", img_pad);
  }
}
