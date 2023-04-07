#include"parameter.h"

namespace navigation
{
    double global_max_Z_adjust;
    double global_min_Z_adjust;
    double current_max_Z_adjust;
    double current_min_Z_adjust;

    double global_map_Resolution;
    double current_map_Resolution;

    string pcd_path;

    void read_param(string path)
    {
        YAML::Node Config = YAML::LoadFile(path);

        global_max_Z_adjust = Config["global_max_z"].as<double>();
        global_min_Z_adjust = Config["global_min_z"].as<double>();

        current_max_Z_adjust = Config["current_max_z"].as<double>();
        current_min_Z_adjust = Config["current_min_z"].as<double>();

        global_map_Resolution = Config["global_map_resolution"].as<double>();
        current_map_Resolution = Config["current_map_resolution"].as<double>();

        pcd_path = path + "../../map/cloud_test.pcd";
    }
}

