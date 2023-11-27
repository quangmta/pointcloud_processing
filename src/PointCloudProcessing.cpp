#include <pointcloud_processing/PointCloudProcessing.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
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
        const float cam_fov_ver,
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
          cam_height_(cam_height) {}

    PointCloudProcessing::~PointCloudProcessing() {}

    void PointCloudProcessing::process_msg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg,
                                           const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg,
                                           const sensor_msgs::msg::PointCloud2::UniquePtr &processed_pointcloud_msg,
                                           const sensor_msgs::msg::LaserScan::UniquePtr &processed_scan_msg)
    {
        // Convert the received message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*pointcloud_msg, *pcl_cloud);

        size_t num_point_within_scan, begin_point_scan_index, end_point_scan_index;
        float cam_angle_max_ = cam_fov_hor_ / 2;
        float coeff_angle = cam_fov_hor_ / (scan_msg->angle_max - scan_msg->angle_min);
        if (coeff_angle < 1)
        {
            num_point_within_scan = std::round(cam_fov_hor_ / scan_msg->angle_increment);
            begin_point_scan_index = std::round((1 - coeff_angle) * scan_msg->ranges.size() / 2);
            end_point_scan_index = std::round((1 + coeff_angle) * scan_msg->ranges.size() / 2);
            float coorY = scan_msg->ranges[end_point_scan_index] * std::tan(cam_angle_max_);
            if (tf_y_ < 0)
            {
                do
                {
                    begin_point_scan_index--;
                    end_point_scan_index--;
                    cam_angle_max_ -= scan_msg->angle_increment;
                } while (std::abs(tf_y_) > coorY - scan_msg->ranges[end_point_scan_index] * std::tan(cam_angle_max_));
            }
            else
            {
                do
                {
                    begin_point_scan_index++;
                    end_point_scan_index++;
                    cam_angle_max_ += scan_msg->angle_increment;
                } while (std::abs(tf_y_) > scan_msg->ranges[end_point_scan_index] * std::tan(cam_angle_max_) - coorY);
            }
        }
        else
        {
            num_point_within_scan = scan_msg->ranges.size();
            begin_point_scan_index = 0;
            end_point_scan_index = scan_msg->ranges.size() - 1;
        }
        
        pcl_cloud->erase(pcl_cloud->begin(), pcl_cloud->begin() + 30 * pcl_cloud->width);
        pcl_cloud->erase(pcl_cloud->begin()+(cam_height_)* pcl_cloud->width, pcl_cloud->end());
        // pcl_cloud->height = cam_height_;

        // Align
        // for (int col = 0; col < cam_width_; col++)
        // {
        //     // find the row
        //     int row_offset = cam_height_ / 2;
        //     // while (!(tf_z_ >= std::min(pcl_cloud->points[(row_offset - 1) * cam_width_ + col].z, pcl_cloud->points[(row_offset + 1) * cam_width_ + col].z) &&
        //     //          tf_z_ <= std::max(pcl_cloud->points[(row_offset - 1) * cam_width_ + col].z, pcl_cloud->points[(row_offset + 1) * cam_width_ + col].z)))
        //     //     row_offset++;
        //     if (tf_z_ < 0)
        //     {
        //         while (tf_z_ < pcl_cloud->points[row_offset * cam_width_ + col].z)
        //         {
        //             row_offset++;
        //             if (row_offset>=cam_height_) break;
        //         }
        //     }
        //     else
        //     {
        //         while (tf_z_ > pcl_cloud->points[row_offset * cam_width_ + col].z)
        //         {
        //             row_offset--;
        //             if (row_offset<0) break;
        //         }
        //     }
        //     auto point = pcl_cloud->points[row_offset * cam_width_ + col];
        //     float coeff_depth = scan_msg->ranges[end_point_scan_index - col * num_point_within_scan / cam_width_] /
        //                   std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));
        //     if (row_offset < cam_height_ && row_offset>=0)
        //     {
        //         for (int row = 0; row < cam_height_; row++)
        //         {
        //             int id = row * cam_width_ + col;
        //             pcl_cloud->points[id].x *= coeff_depth;
        //             pcl_cloud->points[id].y *= coeff_depth;
        //             pcl_cloud->points[id].z *= coeff_depth;
        //         }
        //     }
        // }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

        // // Create the filtering object
        // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        // sor.setInputCloud(pcl_cloud);
        // sor.setLeafSize(0.01f, 0.01f, 0.01f);
        // sor.filter(*cloud_filtered);

        // Convert the processed point cloud back to ROS message format
        pcl::toROSMsg(*pcl_cloud, *processed_pointcloud_msg);

        // Create new scan message with new size
        processed_scan_msg->header = scan_msg->header;
        processed_scan_msg->angle_min = cam_angle_max_ - cam_fov_hor_;
        processed_scan_msg->angle_max = cam_angle_max_;
        processed_scan_msg->angle_increment = scan_msg->angle_increment;
        processed_scan_msg->time_increment = scan_msg->time_increment;
        processed_scan_msg->scan_time = scan_msg->scan_time;
        processed_scan_msg->range_min = scan_msg->range_min;
        processed_scan_msg->range_max = scan_msg->range_max;

        processed_scan_msg->ranges.assign(scan_msg->ranges.begin() + begin_point_scan_index,
                                          scan_msg->ranges.begin() + end_point_scan_index + 1);
    }
}