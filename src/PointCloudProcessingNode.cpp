#include <pointcloud_processing/PointCloudProcessingNode.hpp>

#include <rcutils/logging_macros.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace pointcloud_processing
{
    PointCloudProcessingNode::PointCloudProcessingNode(const rclcpp::NodeOptions &options) : rclcpp::Node("pointcloud_processing_node", options)
    {
        auto qos = rclcpp::SystemDefaultsQoS();

        // Subscribe to the scan (lidar) topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser",
            qos,
            std::bind(&PointCloudProcessingNode::ScanCallback, this, std::placeholders::_1));

        // Subscribe to the input point cloud topic
        point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_in",
            qos,
            std::bind(&PointCloudProcessingNode::PointCloudCallback, this, std::placeholders::_1));

        // Advertise the output point cloud topic
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/processed_scan", qos);
        point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed_points", qos);

        float tf_x = this->declare_parameter("tf_x", 0.05);
        float tf_y = this->declare_parameter("tf_y", 0.02);
        float tf_z = this->declare_parameter("tf_z", -0.05);
        float tf_roll = this->declare_parameter("tf_roll", 0.0);
        float tf_pitch = this->declare_parameter("tf_pitch", 0.0);
        float tf_yaw = this->declare_parameter("tf_yaw", 0.0);
        float cam_fov_hor = this->declare_parameter("cam_fov_hor", 1.210734);
        float cam_fov_ver = this->declare_parameter("cam_fov_ver", 0.742463);
        int cam_width = this->declare_parameter("cam_width", 320);
        int cam_height = this->declare_parameter("cam_height", 180);

        std::string output_frame = this->declare_parameter("output_frame", "/processed_points");

        processed_point_ = std::make_unique<pointcloud_processing::PointCloudProcessing>(tf_x, tf_y, tf_z, tf_roll, tf_pitch,tf_yaw,
                                                                                         cam_fov_hor, cam_fov_ver, cam_width, cam_height);
    }

    PointCloudProcessingNode::~PointCloudProcessingNode() {}

    void PointCloudProcessingNode::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        scan_msg_ = scan;
    }

    void PointCloudProcessingNode::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_msg_)
    {
        if (nullptr == scan_msg_)
        {
            RCLCPP_INFO(get_logger(), "No laser scan info, skipping point cloud processing");
            return;
        }

        try
        {
            sensor_msgs::msg::PointCloud2::UniquePtr processed_point_msg = processed_point_->process_msg(point_msg_, scan_msg_);
            scan_pub_->publish(*scan_msg_);
            point_pub_->publish(std::move(processed_point_msg));
        }
        catch (const std::runtime_error &e)
        {
            RCLCPP_ERROR(get_logger(), "Could not convert depth image to laserscan: %s", e.what());
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_processing::PointCloudProcessingNode)