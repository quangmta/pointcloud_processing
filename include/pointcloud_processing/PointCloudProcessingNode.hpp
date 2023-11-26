#ifndef POINTCLOUD_PROCESSING_NODE
#define POINTCLOUD_PROCESSING_NODE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pointcloud_processing/PointCloudProcessing.hpp>

namespace pointcloud_processing
{
  class PointCloudProcessingNode final: public rclcpp::Node
  {
  public:
    explicit PointCloudProcessingNode(const rclcpp::NodeOptions & options);
    ~PointCloudProcessingNode();

  private:
    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    std::unique_ptr<pointcloud_processing::PointCloudProcessing> processed_point_;
  };
}
#endif