#pragma once
#include<utils/utility.hpp>

namespace navigation
{
    extern double global_max_Z_adjust;
    extern double global_min_Z_adjust;
    extern double current_max_Z_adjust;
    extern double current_min_Z_adjust;

    extern double global_map_Resolution;
    extern double current_map_Resolution;

    extern string pcd_path;

    void read_param(string path);
}

