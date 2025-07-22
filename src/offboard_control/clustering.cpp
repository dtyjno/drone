#include <iostream>
#include <vector>
#include <cmath>
#include "OffboardControl.h"
#include "clustering.h"

using namespace std;
vector<Points> Target_Samples;// 全局变量，存储目标样本点


// 计算两个点的欧几里得距离
double distance(Vector3d a, Vector3d b) 
{
    return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2));
}

// 更新中心点
vector<Vector3d> calculate_center(vector<Points> samples) 
{
    vector<Vector3d> centers(3); // 提前声明
    for (int j = 0; j < 3; j++) 
    {
        double sum_x = 0, sum_y = 0;
        int count = 0;
        for(size_t i = 0; i < samples.size(); i++)
        {
            if (samples[i].cluster_id == j) 
            {
                sum_x += samples[i].point.x();
                sum_y += samples[i].point.y();
                count++;
            }
        }
        if(count > 0) {
            centers[j].x() = sum_x / count;
            centers[j].y() = sum_y / count;
        } else {
            centers[j].setZero(); // 防止除0
        }
    }
    return centers;
}

//从原样本空间中选取三个点作为起始点
vector<Points> Initialize_Clustering(vector<Points> samples)
{
    vector<Points> Center(3);
    Center[0].cluster_id = 0;
    Center[1].cluster_id = 1;
    Center[2].cluster_id = 2;
    Center[0] = samples[0];
    Center[2] = samples[samples.size() - 1];
    Center[1].point = (Center[0].point + Center[2].point) / 2;
    for(size_t i = 0; i < samples.size(); i++)
    {
        Center[0].point = Center[0].point.x() > samples[i].point.x()?samples[i].point:Center[0].point;
        Center[2].point = Center[2].point.x() < samples[i].point.x()?samples[i].point:Center[2].point;
        Center[1].point = (Center[0].point + Center[2].point) / 2;
    }
    //vector<Vector3d> I_Center(3);
    //I_Center[0] = Center[0].point;
    //I_Center[1] = Center[1].point;
    //I_Center[2] = Center[2].point;
    //return I_Center;
    return Center;
}

//聚类算法
vector<Points> Clustering(vector<Points> samples)
{
    vector<Points> Clustering_Result = Initialize_Clustering(samples);
    vector<double>average_dis(3, 0.0);
    vector<vector<double>>sum_diameters(3);
    vector<int> cluster_count(3, 0); // 用于统计每个簇的点数
    // 第一步：分配每个点到最近的簇
    for (size_t i = 0; i < samples.size(); i++ )
    {
        double min_dist = distance(samples[i].point, Clustering_Result[0].point);
        samples[i].cluster_id = 0;
        for(size_t j = 1; j < Clustering_Result.size(); j++)
        {
            double dist = distance(samples[i].point, Clustering_Result[j].point);
            if (min_dist > dist)
            {
                min_dist = dist;
                samples[i].cluster_id = j;
            }
        }
    }
    // 第二步：更新中心点以及计算平均距离
    vector<Vector3d> centers = calculate_center(samples);
    for (size_t j = 0; j < Clustering_Result.size(); j++)
    {
        Clustering_Result[j].point = centers[j]; // 计算中心点
    }
    for(size_t i = 0; i < samples.size(); i++)
    {
        for(size_t j = 0; j < Clustering_Result.size(); j++)
        {
            if(samples[i].cluster_id == static_cast<int>(j))
            {
                average_dis[j] += distance(samples[i].point, Clustering_Result[j].point);
                cluster_count[j]++;
            }
        }
    }
    for(size_t j = 0; j < Clustering_Result.size(); j++)
    {
        if (cluster_count[j] > 0) 
        {
            average_dis[j] /= cluster_count[j]; // 计算平均距离
            Clustering_Result[j].diameters = average_dis[j]; // 更新直径属性
        } 
    }
    //第三步：分配直径
    size_t j = 0;
    for(size_t i = 0; i < samples.size(); i++)
    {
        if (fabs(samples[i].diameters - average_dis[0]) < fabs(samples[i].diameters - average_dis[1]) && fabs(samples[i].diameters - average_dis[0]) < fabs(samples[i].diameters - average_dis[2]))
        {
            j = 0;
        }
        else if (fabs(samples[i].diameters - average_dis[1]) < fabs(samples[i].diameters - average_dis[0]) && fabs(samples[i].diameters - average_dis[1]) < fabs(samples[i].diameters - average_dis[2]))
        {
            j = 1;
        }
        else
        {
            j = 2;
        }
        sum_diameters[j].push_back(samples[i].diameters); // 将最接近的簇的直径设置为目标直径
    }
    for(size_t j = 0; j < Clustering_Result.size(); j++)
    {
        if (sum_diameters[j].empty()) 
        {
            RCLCPP_WARN(rclcpp::get_logger("clustering"), "簇 %zu 没有样本点，无法计算直径", j);
            continue;
        }
        double sum = std::accumulate(sum_diameters[j].begin(), sum_diameters[j].end(), 0.0);
        Clustering_Result[j].diameters = sum / sum_diameters[j].size(); // 计算平均直径
    }
    
    return Clustering_Result;
}