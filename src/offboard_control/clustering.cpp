#include <iostream>
#include <vector>
#include <cmath>
#include "OffboardControl.h"
#include "clustering.h"

std::vector<Circles> Target_Samples;// 全局变量，存储目标样本点

// 计算两个点的欧几里得距离
double distance(Vector3d a, Vector3d b) 
{
    return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2));
}

// 更新中心点
std::vector<Vector3d> calculate_center(std::vector<Circles> samples) 
{
    std::vector<Vector3d> centers(3); // 提前声明
    for (int j = 0; j < 3; ++j)
    {
        double sum_x = 0, sum_y = 0;
        int count = 0;
        for(const auto& sample: samples)
        {
            if (sample.cluster_id == j) 
            {
                sum_x += sample.point.x();
                sum_y += sample.point.y();
                count++;
            }
        }
        if(count > 0) 
        {
            centers[j].x() = sum_x / count;
            centers[j].y() = sum_y / count;
        } 
        else 
        {
            centers[j].setZero(); // 防止除0
        }
    }
    return centers;
}

//从原样本空间中选取三个点作为起始点
std::vector<Circles> Initialize_Clustering(std::vector<Circles> samples)
{
    std::vector<Circles> Center(3);
    Center[0].cluster_id = 0;
    Center[1].cluster_id = 1;
    Center[2].cluster_id = 2;
    Center[0].point = samples[0].point; // 选择第一个点作为第一个中心点
    Center[1].point = samples[samples.size() / 2].point; // 选择第二个点
    Center[2].point = samples[samples.size() - 1].point; // 选择第二个点
    for(const auto& sample: samples)
    {
        double mid_point;
        Center[0].point = sample.point.x() <= Center[0].point.x() ? sample.point : Center[0].point; // 更新第一个中心点
        Center[2].point = sample.point.x() >= Center[2].point.x() ? sample.point : Center[2].point; // 更新第三个中心点
        mid_point = (Center[0].point.x() + Center[2].point.x()) / 2.0; // 计算中点(x轴)
        Center[1].point = fabs(sample.point.x() - mid_point) <= fabs(Center[1].point.x() - mid_point) ? sample.point : Center[1].point; // 更新第二个中心点
    }
    //vector<Vector3d> I_Center(3);
    //I_Center[0] = Center[0].point;
    //I_Center[1] = Center[1].point;
    //I_Center[2] = Center[2].point;
    //return I_Center;
    
    return Center;
}

//聚类算法
std::vector<Circles> Clustering(std::vector<Circles> samples)
{
    std::vector<Circles> Clustering_Result = Initialize_Clustering(samples); //初始化
    std::vector<double>average_dis(3, 0.0);// 用于存储不同簇中点到中心点的平均距离
    std::vector<std::vector<double>>sum_diameters(3);// 存储不同簇读取的直径和，用于后续计算不同簇的平均直径
    std::vector<int> cluster_count(3, 0); // 用于统计每个簇的点数
    // 第一步：分配每个点到最近的簇
    for (auto& sample: samples)
    {
        double min_dist = distance(sample.point, Clustering_Result[0].point);
        sample.cluster_id = 0;
        for(size_t j = 1; j < Clustering_Result.size(); ++j)
        {
            double dist = distance(sample.point, Clustering_Result[j].point);
            if (min_dist > dist)
            {
                min_dist = dist;
                sample.cluster_id = j;
            }
        }
    }
    // 第二步：更新中心点以及计算平均距离
    std::vector<Vector3d> centers = calculate_center(samples);
    for (size_t j = 0; j < Clustering_Result.size(); ++j)
    {
        Clustering_Result[j].point = centers[j]; // 计算中心点
    }
    for(auto& sample: samples)
    {
        for(size_t j = 0; j < Clustering_Result.size(); ++j)
        {
            if(sample.cluster_id == static_cast<int>(j))
            {
                // average_dis[j] += 2 * distance(sample.point, Clustering_Result[j].point);
                average_dis[j] += sample.diameters; // 累加直径
                cluster_count[j]++;
            }
        }
    }
    for(size_t j = 0; j < Clustering_Result.size(); ++j)
    {
        if (cluster_count[j] > 0) 
        {
            average_dis[j] /= cluster_count[j]; // 计算平均距离
            Clustering_Result[j].diameters = average_dis[j]; // 更新直径属性
        } 
    }
    //第三步：分配直径
    // for (const auto& sample : samples)
    // {
    //     double min_diff = std::fabs(sample.diameters - average_dis[0]);
    //     int min_index = 0;

    //     for (int i = 0; i < 3; ++i)
    //     {
    //         double diff = std::fabs(sample.diameters - average_dis[i]);
    //         if (diff < min_diff)
    //         {
    //             min_diff = diff;
    //             min_index = i;
    //         }
    //     }

    //     sum_diameters[min_index].push_back(sample.diameters);
    // }
    // for(size_t j = 0; j < Clustering_Result.size(); ++j)
    // {
    //     if (sum_diameters[j].empty()) 
    //     {
    //         continue;
    //     }
    //     double sum = std::accumulate(sum_diameters[j].begin(), sum_diameters[j].end(), 0.0);
    //     Clustering_Result[j].diameters = sum / sum_diameters[j].size(); // 计算平均直径
    // }
    //按照直径从小到大排序
    // for(auto& result: Clustering_Result)
    // {
    //     Circles swap;
    //     if(Clustering_Result[0].diameters > result.diameters)
    //     {
    //         swap = Clustering_Result[0];
    //         Clustering_Result[0] = result;
    //         result = swap;
    //     }
    //     if(Clustering_Result[2].diameters < result.diameters)
    //     {
    //         swap = Clustering_Result[2];
    //         Clustering_Result[2] = result;
    //         result = swap;
    //     }
    // }

    // sort(Clustering_Result.begin(), Clustering_Result.end(), [](const Circles& a, const Circles& b) {
    //     return a.diameters > b.diameters;
    // });
    
    return Clustering_Result;
}