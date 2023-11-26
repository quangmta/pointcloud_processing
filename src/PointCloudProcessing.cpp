#include <pointcloud_processing/PointCloudProcessing.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pointcloud_processing
{
    PointCloudProcessing::PointCloudProcessing(
      const float tf_x,
      const float tf_y,
      const float tf_z,
      const float tf_roll,
      const float tf_pitch,
      const float tf_yaw,
      const float scan_range_min,
      const float scan_range_max,
      const float scan_angle_min,
      const float scan_angle_max,
      const float cam_fov_hor,
      const int cam_width,
      const int cam_height)
      : tf_x_(tf_x),
      tf_y_(tf_y),
      tf_z_(tf_z),
      tf_roll_(tf_roll),
      tf_pitch_(tf_pitch),
      tf_yaw_(tf_yaw),
      scan_range_min_(scan_range_min),
      scan_range_max_(scan_range_max),
      scan_angle_min_(scan_angle_min),
      scan_angle_max_(scan_angle_max),
      cam_fov_hor_(cam_fov_hor),
      cam_width_(cam_width),
      cam_height_(cam_height){}

    PointCloudProcessing::~PointCloudProcessing(){}

    sensor_msgs::msg::PointCloud2::UniquePtr PointCloudProcessing::process_msg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
        const sensor_msgs::msg::LaserScan::SharedPtr& laser_msg)
    {
        // Convert the received message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromROSMsg(*pointcloud_msg, pcl_cloud);

        // Process the point cloud
        for (auto &point : pcl_cloud.points)
        {
            // Check if the horizontal coordinate is approximately -5cm
            // if (std::abs(point.x - (-0.05)) < 0.001)
            // {
            //     // Change the depth of the point by a factor of 1.5
            //     point.z *= 1.5;
            // }
            point.z *= 1.5;
        }

        // Convert the processed point cloud back to ROS message format
        sensor_msgs::msg::PointCloud2::UniquePtr processed_pc_msg =  std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(pcl_cloud, *processed_pc_msg);
        return processed_pc_msg;
    }

}