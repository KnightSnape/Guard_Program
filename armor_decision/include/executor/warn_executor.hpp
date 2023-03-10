#pragma once
#include<ros/ros.h>
#include<ros/package.h>
#include<fstream>
#include<sstream>
#include<time.h>
#include<string.h>

class Warn_executor
{
    public:
        enum class WarnStatus
        {
            SAFETY,
            DANGEROUS
        };
        typedef std::shared_ptr<Warn_executor> Ptr;


};