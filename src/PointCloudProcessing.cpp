#include <pointcloud_processing/PointCloudProcessing.hpp>

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

    void PointCloudProcessing::AlignLaserScan(const sensor_msgs::msg::LaserScan::UniquePtr &align_scan_msg,
                                              const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)
    {
        size_t begin_point_scan_index, end_point_scan_index;
        float cam_angle_max_ = cam_fov_hor_ / 2;
        float coeff_angle = cam_fov_hor_ / (scan_msg->angle_max - scan_msg->angle_min);

        if (coeff_angle < 1)
        {
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
            begin_point_scan_index = 0;
            end_point_scan_index = scan_msg->ranges.size() - 1;
        }

        // Create new scan message with new size
        align_scan_msg->header = scan_msg->header;
        align_scan_msg->angle_min = cam_angle_max_ - cam_fov_hor_;
        align_scan_msg->angle_max = cam_angle_max_;
        align_scan_msg->angle_increment = scan_msg->angle_increment;
        align_scan_msg->time_increment = scan_msg->time_increment;
        align_scan_msg->scan_time = scan_msg->scan_time;
        align_scan_msg->range_min = scan_msg->range_min;
        align_scan_msg->range_max = scan_msg->range_max;

        align_scan_msg->ranges.assign(scan_msg->ranges.begin() + begin_point_scan_index,
                                      scan_msg->ranges.begin() + end_point_scan_index + 1);
    }

    // void PointCloudProcessing::processPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
    //                                      const sensor_msgs::msg::LaserScan::UniquePtr &scan_msg)
    // {
    //     for (int col = 0; col < cam_width_; col++)
    //     {
    //         // find the row
    //         int row_offset = cam_height_ / 2;
    //         // while (!(tf_z_ >= std::min(pcl_cloud->points[(row_offset - 1) * cam_width_ + col].z, pcl_cloud->points[(row_offset + 1) * cam_width_ + col].z) &&
    //         //          tf_z_ <= std::max(pcl_cloud->points[(row_offset - 1) * cam_width_ + col].z, pcl_cloud->points[(row_offset + 1) * cam_width_ + col].z)))
    //         //     row_offset++;
    //         if (tf_z_ < 0)
    //         {
    //             while (tf_z_ < pcl_cloud->points[row_offset * cam_width_ + col].z)
    //             {
    //                 row_offset++;
    //                 if (row_offset >= cam_height_)
    //                     break;
    //             }
    //         }
    //         else
    //         {
    //             while (tf_z_ > pcl_cloud->points[row_offset * cam_width_ + col].z)
    //             {
    //                 row_offset--;
    //                 if (row_offset < 0)
    //                     break;
    //             }
    //         }
    //         auto point = pcl_cloud->points[row_offset * cam_width_ + col];
    //         float coeff_depth = scan_msg->ranges[scan_msg->ranges.size() - 1 - col * scan_msg->ranges.size() / cam_width_] /
    //                             std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));
    //         if (row_offset < cam_height_ && row_offset >= 0)
    //         {
    //             for (int row = 0; row < cam_height_; row++)
    //             {
    //                 int id = row * cam_width_ + col;
    //                 pcl_cloud->points[id].x *= coeff_depth;
    //                 pcl_cloud->points[id].y *= coeff_depth;
    //                 pcl_cloud->points[id].z *= coeff_depth;
    //             }
    //         }
    //     }
    // }

    void PointCloudProcessing::processPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                                         const sensor_msgs::msg::LaserScan::UniquePtr &scan_msg)
    {
        std::sort(pcl_cloud->points.begin(), pcl_cloud->points.end(), compareX);
        long unsigned int index = 0;
        while (index < pcl_cloud->points.size())
        {
            long unsigned int startId = index;
            long unsigned int offsetId = index;
            while (pcl_cloud->points[index].x - pcl_cloud->points[startId].x < 0.005)
            {
                if (std::abs(pcl_cloud->points[index].y - tf_z_) < std::abs(pcl_cloud->points[offsetId].y - tf_z_))
                {
                    offsetId = index;
                }
                index++;
                if (index >= pcl_cloud->points.size())
                    break;
            }
            float angle = std::atan(pcl_cloud->points[offsetId].x / (pcl_cloud->points[offsetId].z - tf_x_));
            long unsigned int idScan;
            if (angle < -scan_msg->angle_max)
            {
                idScan = scan_msg->ranges.size();
            }
            else if (angle > -scan_msg->angle_min)
            {
                idScan = 0;
            }
            else
            {
                idScan = (-angle - scan_msg->angle_min) / scan_msg->angle_increment;
            }
            float coeff_depth = scan_msg->ranges[idScan] /
                                std::sqrt(std::pow(pcl_cloud->points[offsetId].x + tf_y_, 2) +
                                          std::pow(pcl_cloud->points[offsetId].z - tf_x_, 2));
            for (long unsigned int i = startId; i < index; i++)
            {
                pcl_cloud->points[i].x *= coeff_depth;
                pcl_cloud->points[i].y *= coeff_depth;
                pcl_cloud->points[i].z *= coeff_depth;
            }
        }
    }
    void PointCloudProcessing::process_msg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg,
                                           const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg,
                                           const sensor_msgs::msg::PointCloud2::UniquePtr &processed_pointcloud_msg,
                                           const sensor_msgs::msg::LaserScan::UniquePtr &processed_scan_msg)
    {
        // align laser scan to size of point cloud
        AlignLaserScan(processed_scan_msg, scan_msg);

        // Convert the received message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*pointcloud_msg, *pcl_cloud);

        // Create the filtering object
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
        // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        // sor.setInputCloud(pcl_cloud);
        // sor.setLeafSize(0.005f, 0.005f, 0.005f);
        // sor.filter(*cloud_filtered);

        // Recalculate pointcloud according to laser scan
        // std::sort(cloud_filtered->points.begin(), cloud_filtered->points.end(), compareX);
        processPC(pcl_cloud, processed_scan_msg);

        // Create the filtering object
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(pcl_cloud);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*cloud_filtered);

        // Delete some point to resize the pointcloud to size of rgb image
        // cloud_filtered->erase(cloud_filtered->begin(), cloud_filtered->begin() + (int)(cloud_filtered->points.size()/2));
        // pcl_cloud->erase(pcl_cloud->begin() + cam_height_*cam_height_, pcl_cloud->end());
        // pcl_cloud->height = cam_height_;

        // Convert the processed point cloud back to ROS message format
        pcl::toROSMsg(*cloud_filtered_2, *processed_pointcloud_msg);
    }
}