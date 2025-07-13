#include <iostream>
#include <vector>
#include <cmath>
#include "OffboardControl.h"
#include "clustering.h"

using namespace std;

class Circle
{
public:
    double m_x,m_y;
    int m_circle_id;
}

struct Points
{
    double x,y;
    int cluster_id;
}

// 计算两个点的欧几里得距离
double distance(const Point& a, const Point& b) 
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// 更新中心点
Vector3d calculate_center(vector<std::optional<Vector3d>> samples) 
{
    double sum_x = 0, sum_y = 0;
    int count = 0;
    for (const auto& p : points) 
    {
        if (p.cluster_id == cluster_id) 
        {
            sum_x += p.x;
            sum_y += p.y;
            ++count;
        }
    }
    return { sum_x / count, sum_y / count, cluster_id };
}

//从原样本空间中选取三个点作为起始点
vector<Vector3d> Initialize_Clustering(vector<std::optional<Vector3d>> samples)
{
    int i;
    vector<Vector3d> Center(3);
    for(i  = 0;i < vector.size(); i++)
    {
        Center[0] = Center[0].x() > samples[i].x()?samples[i]:Center[0];
        Center[2] = Center[2].x() < samples[i].x()?samples[i]:Center[2];
        Center[1] = (Center[0] + Center[2]) / 2;
    }

    
    return Center;
}

//聚类算法
Vector3d Clustering(vector<std::optional<Vector3d>> samples)
{
    First_center = Initialize_Clustering(samples);
    for(int j = 0;j < samples.size();J ++)
    {
        if(samples[j] == nullopt)
        {
            samples[j] = 0;
        }
    }
    Vector3d clustering_result;

    // 第一步：分配每个点到最近的簇
    for (auto& p : points) 
    {
        double min_dist = 1e9;
        int best_cluster = -1;
        for (const auto& c : centers) 
        {
            double d = distance(p, c);
            if (d < min_dist) 
            {
                min_dist = d;
                best_cluster = c.cluster_id;
            }
        }
        p.cluster_id = best_cluster;
    }

    // 第二步：更新中心点
    for (int i = 0; i < centers.size(); ++i) 
    {
        centers[i] = calculate_center(points, i);
    }

    return clustering_result;
}