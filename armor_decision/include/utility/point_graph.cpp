#include"point_graph.hpp"
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
//构造路径
void Graph::build_path()
{
    shortest_path_node.resize(nodes.size());
    for(int i=0;i<shortest_path_node.size();i++)
    {
        shortest_path_node[i].resize(nodes.size());
    }

    for(auto &&start_node : nodes) {
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