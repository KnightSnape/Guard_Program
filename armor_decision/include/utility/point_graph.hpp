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

    void initParam();

    void add_node(string node_name,Point node_place,vector<Point> node_area);

    void add_edge(int first,int next,int weight);

    void build_path();

    int get_first_point(int start_id,int final_id);

    int CheckInPoint(Point point);

    int findLatestpoint(Point point);

    void main_process();

private:

    vector<Graph_Node> nodes;
    vector<vector<vector <int> > > shortest_path_node;
    vector<int> shortest_path_sequence;

    inline double Distance(const Point& A,const Point& B)
    {
        return sqrt((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y));
    }

};

