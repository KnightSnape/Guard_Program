#include"point_graph.hpp"

void Graph::initParam(std::string path)
{
    std::string graph_point_path = path + "graph_point.yaml";
    std::string graph_edge_path = path + "edge.yaml";
    std::string graph_point_area_path = path + "map_point_area.yaml";
    YAML::Node Graph_Point_Config = YAML::LoadFile(graph_point_path);
    YAML::Node Graph_Edge_Config = YAML::LoadFile(graph_edge_path);
    YAML::Node Graph_Point_Area_Config =YAML::LoadFile(graph_point_area_path);
    int point_cnt = Graph_Point_Config["point_cnt"].as<int>();
    for(int i=0;i<point_cnt;i++)
    {
        vector<int> get_center_point = Graph_Point_Config["ID" + std::to_string(i) + "point"].as<std::vector<int> >();
        cv::Point center_point = vector_to_point(get_center_point);
        vector<vector<int> > get_area_points = Graph_Point_Area_Config["point" + std::to_string(i) + "Area"].as<std::vector<std::vector<int> > >();
        vector<Point> node_area = vector_to_point(get_area_points);
        add_node(std::to_string(i),center_point,node_area);
    }
    int edge_cnt = Graph_Edge_Config["edge_cnt"].as<int>();
    for(int i=0;i<edge_cnt;i++)
    {
        vector<int> get_node_vector = Graph_Edge_Config["edge" + std::to_string(i+1)].as<std::vector<int>>();
        add_edge(get_node_vector[0]-1,get_node_vector[1]-1,get_node_vector[2]);
    }
    ROS_DEBUG("build_node done");
    build_path();
    ROS_DEBUG("build shortest path done");
}
//增加节点
void Graph::add_node(string node_name,Point node_place,vector<Point> node_area)
{
    Graph_Node node;
    node.node_id = nodes.size();
    node.node_name = node_name;
    node.node_place = node_place;
    node.node_area = node_area;
    nodes.push_back(node);
}
//增加边
void Graph::add_edge(int first,int next,int weight)
{
    nodes[first].next_node_map.emplace_back(next,weight);
}

Point Graph::get_node_point(int node)
{
    return nodes[node].node_place;
}
//构造路径
void Graph::build_path()
{
    shortest_path_node.resize(nodes.size());
    for(int i=0;i<shortest_path_node.size();i++)
    {
        shortest_path_node[i].resize(nodes.size());
    }

    for(auto &&start_node : nodes) 
    {
        //以start_node为起点时，到达v最短的时候v的前驱动
        int pre[nodes.size()];
        int vis[nodes.size()];
        memset(vis, 0, sizeof(vis));
        int dis[nodes.size()];
        memset(dis, 0x3f, sizeof(dis));
        dis[start_node.node_id] = 0;
        for (int i = 0; i < nodes.size(); i++) {
            int u = 0, mind = 0x3f3f3f3f;
            for (int j = 0; j < nodes.size(); j++) {
                if (!vis[j] && dis[j] < mind)
                    u = j, mind = dis[j];
            }
            vis[u] = true;
            for (auto ed: nodes[u].next_node_map) {
                int v = ed.first, w = ed.second;
                if (dis[v] > dis[u] + w) {
                    dis[v] = dis[u] + w;
                    pre[v] = u;
                }
            }
        }

        for (int i = 0; i < nodes.size(); i++) {
            vector<int> temp_vector;
            int temp = i;
            while (temp != start_node.node_id) {
                temp_vector.push_back(temp);
                temp = pre[temp];
            }
            temp_vector.push_back(start_node.node_id);
            shortest_path_node[start_node.node_id][i] = temp_vector;
        }
    }
}

void Graph::check_edge(int node)
{
    for(int i=0;i<nodes[node].next_node_map.size();i++)
    {
        ROS_DEBUG("get next_weight is %d,weight is %d",nodes[node].next_node_map[i].first,nodes[node].next_node_map[i].second);
    }
}

void Graph::check_shortest_path(int node1,int node2)
{
    ROS_DEBUG("start checking shortest path");
    for(int i=0;i<shortest_path_node[node1][node2].size();i++)
    {
        ROS_DEBUG("node %d: %d",i+1,shortest_path_node[node1][node2][i]);
    }
}
//得到第一个点
int Graph::get_first_point(int start_id,int final_id)
{
    int first_id;
    if(shortest_path_node[start_id][final_id].size() < 2)
        first_id = start_id;
    else
        first_id = shortest_path_node[start_id][final_id][shortest_path_node[start_id][final_id].size()-2];
    return first_id;
}
//坐标是否再多边形内
int Graph::CheckInPoint(Point point)
{
    //遍历ID
    for(int i=0;i<nodes.size();i++)
    {
        if(pointPolygonTest(nodes[i].node_area,point,false) == 1)
        {
            //返回该点的ID
            return i;
        }
    }
    //错误
    return -1;
}
//当且仅当点在多边形外面(不准确画框容易出现这个问题)
int Graph::findLatestpoint(Point point)
{
    double distance_min = 1000000.0;
    int p = -1;
    for(int i=0;i<nodes.size();i++)
    {
        auto distance = Distance(nodes[i].node_place,point);
        if(distance < distance_min)
        {
            distance_min = distance;
            p = nodes[i].node_id;
        }
    }
    return p;
}
//是否已经到达局部目标附近
bool Graph::CheckOnTarget(Point point,int target_id)
{
    cv::Point target_point = nodes[target_id].node_place;
    return Distance(point,target_point) < 4;
}
//更新序列点
void Graph::updatePoint(Point point,int &start_id,int target_id,int &target_state,int &next_target)
{
    //获取next_point点，即下一个点
    target_state = 0;
    //如果起始点和终点完全一致且
    if(start_id == target_id && CheckInPoint(point) == target_id)
    {   
        target_state = 1;
        return;
    }
    //如果已经到达某一个位点，更新
    if(CheckOnTarget(point,next_target) && next_target != target_id)
    {
        start_id = next_target;
        next_target = get_first_point(next_target,target_id);
        target_state = 2;
    }
    //如果已经完全到达该位置，直接显示已经到达
    else if(CheckOnTarget(point,next_target) && next_target == target_id)
    {
        ROS_DEBUG("Get Target");
        target_state = 1;
    }

}  


