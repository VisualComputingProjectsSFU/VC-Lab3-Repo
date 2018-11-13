//
// Created by rakesh on 27/08/18.
//

#include <icp_slam/mapper.h>
#include <icp_slam/utils.h>
#include <string>
using namespace std;

namespace icp_slam
{
// 

  Mapper::Mapper(){
    //nothing
  };
  

void Mapper::initMap(int width, int height, float resolution,double origin_x_meters, double origin_y_meters,
               uint8_t *pointer, unsigned char unknown_cost_value)
{
	//TODO: intialize the map
  

  width_=width;
  height_=height;
  resolution_=resolution;  ///< @brief meters/pixels

  map_ = cv::Mat(height,width,CV_8SC1);

  origin_x_=origin_x_meters ; ///< origin of the map in meters
  origin_y_=origin_y_meters; ///< origin of the map in meters

  init_robot_pose_.x=0;
  init_robot_pose_.y=0;
  init_robot_pose_.a=0;

  init_robot_grid_pts = cv::Point2d(origin_x_,origin_y_);

  for(int i=0;i<height;i++)
  {
  	for(int j=0;j<width;j++)
  	{
  		map_.at<int8_t>(i,j)=(int8_t)NO_INFORMATION;
  	}
  }

  is_initialized_ = true;

  relative_map_ = cv::Mat::zeros(height,width,CV_8SC1);

}

Mapper::robot_pose_t Mapper::poseFromTf(const tf::StampedTransform &tf_pose)
{
  Mapper::robot_pose_t robot_pose_;
  robot_pose_.x=(double)tf_pose.getOrigin().getX();
	robot_pose_.y=(double)tf_pose.getOrigin().getY();
	robot_pose_.a=(double)tf::getYaw(tf_pose.getRotation());
}

cv::Point2d Mapper::getRobotpoints(robot_pose_t &init_pose,
							robot_pose_t &current_pose)
{
	cv::Point2d robot;
	double point_x=(double)round(current_pose.x+init_pose.x);
	double point_y=(double)round(current_pose.y+init_pose.x);
	robot=cv::Point2d(point_x,point_y);

	return robot; 
}

// float Mapper::normalizeTo360Angle(float &angle)
//   {
//     auto out_angle = std::fmod(angle, 2*M_PI);
//     if (out_angle < 0)
//     {
//       out_angle += 2*M_PI;
//     }
//     return out_angle;
//   }

cv::Mat Mapper::getLaserpoints(const sensor_msgs::LaserScanConstPtr &laser_scan_ptr,
                               const Mapper::robot_pose_t &current_pose)
{
  double rotation = current_pose.a;
  sensor_msgs::LaserScan laser_scan = *laser_scan_ptr;
  auto range_size = (laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment ;
  cv::Mat point_mat = cv::Mat(1,2,CV_32F);
  cv::Mat temp_mat = cv::Mat(1,2,CV_32F);
  float x, y;
  bool normalise;
  auto teta = laser_scan.angle_min + rotation;
  if (laser_scan.angle_min!=0)
  {
    normalise = true;
  }
  int idx=0;
  //int row_cnt=0;
  for(int i=0;i<range_size;i++)
  {
    teta = teta + laser_scan.angle_increment;
    if (normalise)
    {
      teta = normalizeTo360Angle(teta);
    }
    if (teta>2*M_PI)
    {
      teta = laser_scan.angle_increment;
    }
    ///ROS_INFO("ranges : (%i),angle %f  is %f ",i,teta,laser_scan.ranges[i]);
    if((laser_scan.ranges[i]>laser_scan.range_min)&&(laser_scan.ranges[i]<laser_scan.range_max))
    {
      utils::polarToCartesian(laser_scan.ranges[i], teta , x, y);
      if (idx==0)
      {
        point_mat.at<float>(i, 0)=round(x);
        point_mat.at<float>(i, 1)=round(y);

      }
      else
      {
        temp_mat.at<float>(0, 0)=round(x);
        temp_mat.at<float>(0, 1)=round(y);
        point_mat.push_back(temp_mat);
      }
      idx++;
    }
  }
  return point_mat;
}

int Mapper::updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan,
                      const tf::StampedTransform &pose)
{
  Mapper::robot_pose_t current_pose= Mapper::poseFromTf(pose);
  cv::Point2d robot_points = Mapper::getRobotpoints(init_robot_pose_,current_pose);
  cv::Point2d robot_grid_points = init_robot_grid_pts + robot_points;

  cv::Mat laser_points = Mapper::getLaserpoints(laser_scan,current_pose);

  int nrows = laser_points.rows;

  for(int i=0;i<nrows;i++)
  {
    cv::Point2d laser_pt = cv::Point2d(laser_points.at<float>(i,0),laser_points.at<float>(i,1));
    
    cv::Point2d laser_grid_pt = robot_grid_points+laser_pt;

    //cout<<laser_grid_pt<<endl;

    cv::LineIterator it(map_,robot_grid_points,laser_grid_pt,8);
    for(int i = 0; i < it.count; i++, ++it) 
    {
      cv::Point point = it.pos(); // (point.x, point.y)
      if(i==(it.count-1))
      {
       relative_map_.at<int8_t>(point.x,point.y)=relative_map_.at<int>(point.x,point.y)+1; 
      }
      else
      {
      relative_map_.at<int8_t>(point.x,point.y)=relative_map_.at<int8_t>(point.x,point.y)-1; 
      }
      if (relative_map_.at<int8_t>(point.x,point.y) > 1)
      {
        map_.at<int8_t>(point.x,point.y)=(int8_t)LETHAL_OBSTACLE;
      }
      else if(relative_map_.at<int8_t>(point.x,point.y) < -1)
      {
        map_.at<int8_t>(point.x,point.y)=(int8_t)FREE_SPACE;
      }
      else
      {
        map_.at<int8_t>(point.x,point.y)=(int8_t)NO_INFORMATION;
      }
    }
  }
}

cv::Mat Mapper::getMapCopy()
{
  cv::Mat out_map = cv::Mat(height_,width_,CV_8SC1);

  for (int i=0;i<height_;i++)
  {
    for(int j=0;j<width_;j++)
    {
      out_map.at<int8_t>(i,j)=map_.at<int8_t>(i,j);
    }
  }

  cout<<out_map.size()<<endl;

  return out_map;
}



} // namespace icp_slam
