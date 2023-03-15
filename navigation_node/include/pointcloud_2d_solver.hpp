#pragma once
#include<Eigen/Eigen>
#include<pcl/io/vtk_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/OccupancyGrid.h>

class PointCloud_2d_Solver
{
    public:
        PointCloud_2d_Solver()
        {
            //LaserScanInit();
        }

        void LaserScanInit()
        {
            laser_scan.header.frame_id = "camera_init";
            laser_scan.angle_min = -M_PI/2.0;
            laser_scan.angle_max = M_PI/2.0;
            laser_scan.angle_increment = M_PI/180.0;
            laser_scan.time_increment=0.0;
            laser_scan.scan_time=0.1;
            laser_scan.range_min=0.0;
            laser_scan.range_max=100.0;
        }

        void get_pointcloud(const sensor_msgs::PointCloud2 point_cloud)
        {
            cloud_get.data = point_cloud.data;
            cloud_get.fields = point_cloud.fields;
            cloud_get.height = point_cloud.height;
            cloud_get.is_bigendian = point_cloud.is_bigendian;
            cloud_get.is_dense = point_cloud.is_dense;
            cloud_get.row_step = point_cloud.row_step;
            cloud_get.width = point_cloud.width;
            cloud_get.point_step = point_cloud.point_step;
            cloud_get.header.stamp = ros::Time::now();
            cloud_get.header.frame_id = "camera_init";
        }
        /// @brief 下采样
        void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,pcl::PointCloud<pcl::PointXYZ>& downsampled_cloud,float voxel_size)
        {
            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud(input_cloud);
            sor.setLeafSize(voxel_size,voxel_size,voxel_size);
            sor.filter(downsampled_cloud);
        }

        void convertToOccupyGrid(pcl::PointCloud<pcl::PointXYZ> cloud_get,nav_msgs::OccupancyGrid& occupancy_grid,float resolution)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = cloud_get.makeShared();

            occupancy_grid.info.resolution = resolution;

            int width = std::ceil((input_cloud->width * resolution) / occupancy_grid.info.resolution);
            int height = std::ceil((input_cloud->height * resolution) / occupancy_grid.info.resolution);

            occupancy_grid.info.width = width;
            occupancy_grid.info.height = height;

            occupancy_grid.info.origin.position.x = -(width / 2) * occupancy_grid.info.resolution;
            occupancy_grid.info.origin.position.y = -(width / 2) * occupancy_grid.info.resolution;
            occupancy_grid.info.origin.orientation.w = 1;

            occupancy_grid.data.resize(width*height,-1);

            for(size_t i=0;i<input_cloud->points.size();i++)
            {
                int x = (input_cloud->points[i].x - occupancy_grid.info.origin.position.x) / resolution;
                int y = (input_cloud->points[i].y - occupancy_grid.info.origin.position.y) / resolution;

                // If point is within grid bounds, set corresponding grid cell to occupied (100)
                if (x >= 0 && x < width && y >= 0 && y < height)
                {
                    int index = y * width + x;
                    occupancy_grid.data[index] = 100;
                }
            }
        }

        nav_msgs::OccupancyGrid PointCloud2ToGrid()
        {
            pcl::fromROSMsg(cloud_get,cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
            cloud_ptr = cloud.makeShared();
            pcl::PointCloud<pcl::PointXYZ> downSampledCloud;
            //TODO(knight): use config.yaml
            downsamplePointCloud(cloud_ptr,downSampledCloud,0.1);
            convertToOccupyGrid(downSampledCloud,grid_msg,0.1);
            return grid_msg;
        }

        sensor_msgs::LaserScan get_laser_scan()
        {
            return laser_scan;
        }
    private:
        pcl::PointCloud<pcl::PointXYZ> cloud;
        sensor_msgs::PointCloud2 cloud_get;
        sensor_msgs::LaserScan laser_scan;
        nav_msgs::OccupancyGrid grid_msg;

};