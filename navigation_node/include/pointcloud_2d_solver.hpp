#pragma once
#include"utils/utility.hpp"
#include"parameter.h"

namespace navigation
{

class PointCloud_2d_Solver
{
    public:
        PointCloud_2d_Solver()
        {
            this->get_solver_param();
            //this->global_map_init();
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

        void get_solver_param()
        {
            this->global_max_z_adjust = global_max_Z_adjust;
            this->global_min_z_adjust = global_min_Z_adjust;
            this->current_max_z_adjust = current_max_Z_adjust;
            this->current_min_z_adjust = current_min_Z_adjust;

            this->global_resolution = global_map_Resolution;
            this->current_resolution = current_map_Resolution;
        }

        int global_map_init()
        {
            if(pcl::io::loadPCDFile(pcd_path,global_cloud) == -1)
            {
                PCL_ERROR("Couldn't read file: %s \n",pcd_path.c_str());
                return (-1);
            }
            find_Z_value(global_cloud.makeShared());
            PassThroughFilter(global_cloud.makeShared(),global_cloud,false,0,0);
            PassThroughFilter(global_cloud.makeShared(),global_cloud_filtered,false,global_max_z_adjust,global_min_z_adjust);
        }

        void find_Z_value(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
        {
            max_z = 0;
            min_z = 100000.0;
            for(int i = 0; i < cloud_data->points.size() - 1; i++)
            {
                if(cloud_data->points[i].z > max_z)
                {
                    max_z = cloud_data->points[i].z;
                }
                if(cloud_data->points[i].z < min_z)
                {
                    min_z = cloud_data->points[i].z;
                }
            }
        }
        
        void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,pcl::PointCloud<pcl::PointXYZ>& downsampled_cloud,const bool &flag_in,double z_max,double z_min)
        {
            pcl::PassThrough<pcl::PointXYZ> passthrough;
            passthrough.setInputCloud(input_cloud);
            passthrough.setFilterFieldName("z");
            passthrough.setFilterLimits(min_z+z_min, max_z-z_max);
            passthrough.setFilterLimitsNegative(flag_in);
            passthrough.filter(downsampled_cloud);
        }

        void convertToOccupyGrid(pcl::PointCloud<pcl::PointXYZ> cloud,nav_msgs::OccupancyGrid& occupancy_grid,double resolution)
        {
            occupancy_grid.header.seq = 0;
            occupancy_grid.header.stamp = ros::Time::now();
            occupancy_grid.header.frame_id = "camera_init";

            occupancy_grid.info.map_load_time = ros::Time::now();
            occupancy_grid.info.resolution = resolution;

            double x_min,x_max,y_min,y_max;
            double z_max_grey_rate = 0.05;
            double z_min_grey_rate = 0.95;
            double k_line = (z_max_grey_rate - z_min_grey_rate) / (max_z - min_z);
            double b_line = (max_z * z_min_grey_rate - min_z * z_max_grey_rate) / (max_z - min_z);

            if(cloud.points.empty())
            {
                ROS_WARN("pcd is empty!\n");
                return;
            }

            for(int i=0;i<cloud.points.size()-1;i++)
            {
                if(i==0)
                {
                    x_min = x_max = cloud.points[i].x;
                    y_min = y_max = cloud.points[i].y;
                }
                double x = cloud.points[i].x;
                double y = cloud.points[i].y;

                if(x < x_min) x_min = x;
                if(x > x_max) x_max = x; 

                if(y < y_min) y_min = y;
                if(y > y_max) y_max = y;
            }

            occupancy_grid.info.origin.position.x = x_min;
            occupancy_grid.info.origin.position.y = y_min;
            occupancy_grid.info.origin.position.z = 0.0;
            occupancy_grid.info.origin.orientation.x = 0.0;
            occupancy_grid.info.origin.orientation.y = 0.0;
            occupancy_grid.info.origin.orientation.z = 0.0;
            occupancy_grid.info.origin.orientation.w = 1.0;

            occupancy_grid.info.width = int((x_max - x_min) / resolution);
            occupancy_grid.info.height = int((y_max - y_min)  / resolution);

            occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height);
            occupancy_grid.data.assign(occupancy_grid.info.width * occupancy_grid.info.height,0);

            for(int iter = 0;iter < cloud.points.size();iter++)
            {
                    int i = int((cloud.points[iter].x - x_min) / resolution);
                    if(i < 0 || i >= occupancy_grid.info.width) continue;

                    int j = int((cloud.points[iter].y - y_min) / resolution);
                    if(j < 0 || j >= occupancy_grid.info.height - 1) continue;

                    occupancy_grid.data[i + j * occupancy_grid.info.width] = 100;
            }

        }

        nav_msgs::OccupancyGrid PointCloud2ToGrid()
        {
            pcl::fromROSMsg(cloud_get,cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
            cloud_ptr = cloud.makeShared();
            pcl::PointCloud<pcl::PointXYZ> SampledCloud;
            find_Z_value(cloud_ptr);
            PassThroughFilter(cloud_ptr,SampledCloud,false,current_max_z_adjust,current_min_z_adjust);
            convertToOccupyGrid(SampledCloud,grid_msg,current_resolution);
            return grid_msg;
        }

        sensor_msgs::LaserScan get_laser_scan()
        {
            return laser_scan;
        }

        sensor_msgs::PointCloud2 get_downsampled_pointcloud2()
        {
            return down_sample_point_cloud;
        }

    private:
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> global_cloud;
        pcl::PointCloud<pcl::PointXYZ> global_cloud_filtered;
        sensor_msgs::PointCloud2 cloud_get;
        sensor_msgs::LaserScan laser_scan;
        sensor_msgs::PointCloud2 down_sample_point_cloud;
        nav_msgs::OccupancyGrid grid_msg;

        double max_z;
        double min_z;

        double grid_x = 0.1;
        double grid_y = 0.1;
        double grid_z = 0.1;

        double thre_radius = 0.5;

        double global_max_z_adjust;
        double global_min_z_adjust;
        double current_max_z_adjust;
        double current_min_z_adjust;

        double global_resolution;
        double current_resolution;

};

}