#include <pointcloud_processing/PointCloudProcessing.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>

namespace pointcloud_processing
{
    PointCloudProcessing::PointCloudProcessing(
      const float tf_x,
      const float tf_y,
      const float tf_z,
      const float tf_roll,
      const float tf_pitch,
      const float tf_yaw,
      const float cam_fov_hor,
      const float cam_fov_ver;
      const int cam_width,
      const int cam_height)
      : tf_x_(tf_x),
      tf_y_(tf_y),
      tf_z_(tf_z),
      tf_roll_(tf_roll),
      tf_pitch_(tf_pitch),
      tf_yaw_(tf_yaw),
      cam_fov_hor_(cam_fov_hor),
      cam_fov_ver_(cam_fov_ver),
      cam_width_(cam_width),
      cam_height_(cam_height){}

    PointCloudProcessing::~PointCloudProcessing(){}

    sensor_msgs::msg::PointCloud2::UniquePtr PointCloudProcessing::process_msg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
        const sensor_msgs::msg::LaserScan::SharedPtr& laser_msg)
    {
        // number of point: laser_msg.ranges.size();
        // angle_min, angle_max, angle_increment, range_min, range_max        
        int begin_point_scan_index, end_point_scan_index,num_point_within_scan;
        float coeff = cam_fov_for_/(laser_msg->angle_max-laser_msg->angle_min)
        if (coeff <1)
        {
            num_point_within_scan = (int)(cam_fov_for_/laser_msg->angle_increment);
            begin_point_scan_index = ((1-coeff)*laser_msg->ranges.size())/2;
            end_point_scan_index = ((1+coeff)*laser_msg->ranges.size())/2;
        }
        else
        {
            num_point_within_scan = laser_msg->ranges.size();
            begin_point_scan_index = 0;
            end_point_scan_index = laser_msg->ranges.size()-1;
        }        

        // Convert the received message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromROSMsg(*pointcloud_msg, pcl_cloud);

        for (int i =0; i<pcl_cloud.width;i++)
        {
            
        }



        // // Process the point cloud
        // for (auto &point : pcl_cloud.points)
        // {
        //     // Check if the vertical coordinate is tf_z
        //     if (std::abs(point.z - (tf_z_)) < point.x*(std::tan2(cam_fov_ver_/2))/cam_height_)
        //     {                
        //         point.x = laser_msg->range[];
        //     }
        // }

        // Convert the processed point cloud back to ROS message format
        sensor_msgs::msg::PointCloud2::UniquePtr processed_pc_msg =  std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(pcl_cloud, *processed_pc_msg);
        return processed_pc_msg;
    }

}