#ifndef LOG_EXECUTOR_H
#define LOG_EXECUTOR_H


#include<ros/ros.h>
#include<ros/package.h>
#include<fstream>
#include<sstream>
#include<time.h>
#include<string.h>
#include<yaml-cpp/yaml.h>

class Log_executor
{
    public:
        typedef std::shared_ptr<Log_executor> Ptr;
        Log_executor():create_(false)
        {
            time_t tt;
            time(&tt);
            tt = tt + 8*3600;
            tm* t = gmtime(&tt);
            char str[30];
            sprintf(str, "/log/%d-%02d-%02d %02d:%02d:%02d.txt",
                t->tm_year + 1900,
                t->tm_mon + 1,
                t->tm_mday,
                t->tm_hour, 
                t->tm_min,
                t->tm_sec);
        }
        ~Log_executor(){}
        void print(std::stringstream& str_stream)
        {
            if(!create_)
                return;
        }
    
    private:
        std::ofstream outFile;
        bool create_;
};
#endif