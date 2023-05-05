#pragma once

#include<iostream>
#include<vector>
#include<chrono>
#include"utility/utility.hpp"

using namespace std;

class Graph
{
public:
    Graph()=default;
    void initParam(std::string path);
    void add_node(string node_name,Point node_place,vector<Point> node_area);
    void add_edge(int first,int next,int weight);
    Point get_node_point(int node);
    void check_edge(int node);
    void check_shortest_path(int node1,int node2);
    void build_path();
    int get_first_point(int start_id,int final_id);
    int CheckInPoint(Point point);
    int findLatestpoint(Point point);
    bool CheckOnTarget(Point point,int target_id);
    void updatePoint(Point point,int &start_id,int target_id,int &target_state,int &next_target);
    void main_process();

private:

    vector<Graph_Node> nodes;
    vector<vector<vector <int> > > shortest_path_node;
    vector<int> shortest_path_sequence;

    inline double Distance(const Point& A,const Point& B)
    {
        return sqrt((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y));
    }

    inline cv::Point vector_to_point(const std::vector<int>& point)
    {   
        cv::Point final_point = cv::Point(point[0],point[1]);
        return final_point;
    }

    inline std::vector<cv::Point> vector_to_point(const std::vector<std::vector<int> >& point)
    {
        std::vector<cv::Point> final_points;
        for(int i=0;i<point.size();i++)
        {
            final_points.push_back(vector_to_point(point[i]));
        }
        return final_points;
    }

};

