#ifndef POINTCLOUD_PROCESSING
#define POINTCLOUD_PROCESSING

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace pointcloud_processing
{
  class PointCloudProcessing final
  {
  public:
    explicit PointCloudProcessing(
      const float tf_x,
      const float tf_y,
      const float tf_z,
      const float tf_roll,
      const float tf_pitch,
      const float tf_yaw,
      const float cam_fov_hor,
      const int cam_width,
      const int cam_height
      );
    ~PointCloudProcessing();

    sensor_msgs::msg::PointCloud2::UniquePtr process_msg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
                                                         const sensor_msgs::msg::LaserScan::SharedPtr& laser_msg);

  private: 
    float tf_x_;
    float tf_y_;
    float tf_z_;
    float tf_roll_;
    float tf_pitch_;
    float tf_yaw_;    
    float cam_fov_hor_;
    int cam_width_;
    int cam_height_;
    
  };
}
#endif