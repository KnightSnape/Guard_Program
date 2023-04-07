#pragma once
#include<chrono>
#include<geometry_msgs/TransformStamped.h>

namespace navigation
{

class TF
{
    public:
        TF()
        {

        }
        geometry_msgs::TransformStamped mapping_TF_solver()
        {
            geometry_msgs::TransformStamped transform;
            transform.header.frame_id = "map";
            transform.child_frame_id = "camera_init";
            transform.transform.translation.x = 0;
            transform.transform.translation.y = 0;
            transform.transform.translation.z = 0;
            transform.transform.rotation.w = 1.0;
            transform.transform.rotation.x = 0;
            transform.transform.rotation.y = 0;
            transform.transform.rotation.z = 0;
            transform.header.stamp = ros::Time::now();
            return transform;
        }
        geometry_msgs::TransformStamped base_link_TF_solver()
        {
            geometry_msgs::TransformStamped transform;
            transform.header.frame_id = "base_link";
            transform.child_frame_id = "map";
            transform.transform.translation.x = 0;
            transform.transform.translation.y = 0;
            transform.transform.translation.z = 0;
            transform.transform.rotation.w = 1.0;
            transform.transform.rotation.x = 0;
            transform.transform.rotation.y = 0;
            transform.transform.rotation.z = 0;
            transform.header.stamp = ros::Time::now();
            return transform;
        }

    private:

};

}

