#include <iostream>
#include <vector>
#include <cmath>
#include "OffboardControl.h"
#include "clustering.h"

std::vector<Circles> Target_Samples;// 全局变量，存储目标样本点

// 计算两个三维点的欧几里得距离
double distance(Circles a, Circles b) 
{
    return sqrt(pow(a.point.x() - b.point.x(), 2) + pow(a.point.y() - b.point.y(), 2) + pow(a.diameters - b.diameters, 2));
}

//计算两个一维点的绝对距离
double AbsoluteDistance(double a, double b) 
{
    return std::fabs(a - b);
}

// 更新中心点
std::vector<Circles> calculate_center(std::vector<Circles> circles_sample) 
{
    std::vector<Circles> centers(3); // 提前声明
    for (int j = 0; j < 3; ++j)
    {
        double sum_x = 0, sum_y = 0, sum_d = 0;
        int count = 0;
        for(const auto& circle: circles_sample)
        {
            if (circle.cluster_id == j) 
            {
                sum_x += circle.point.x();
                sum_y += circle.point.y();
                sum_d += circle.diameters; // 累加直径
                count++;
            }
        }
        if(count > 0)
        {
            centers[j].point.x() = sum_x / count;
            centers[j].point.y() = sum_y / count;
            centers[j].diameters = sum_d / count; 
        } 
        else 
        {
            centers[j].point.setZero();
            centers[j].diameters = 0; // 防止除0
        }
    }
    return centers;
}

//从原样本空间中选取三个点作为起始点
std::vector<Circles> Initialize_Clustering(std::vector<Circles> samples)
{
    std::vector<Circles> Center(3);
    for(size_t i = 0;i < Center.size(); ++i)
    {
        Center[i].cluster_id = i; // 初始化簇ID
        Center[i].diameters = 0; // 初始化直径为0
    }
    Center[0].point = samples[0].point; 
    Center[1].point = samples[samples.size() / 2].point; 
    Center[2].point = samples[samples.size() - 1].point; 
    for(const auto& sample: samples)
    {
        double mid_point;
        if(sample.point.x() <= Center[0].point.x())
        {
            Center[0].point = sample.point; // 更新第一个中心点
            Center[0].diameters = sample.diameters; // 更新直径
        }
        if(sample.point.x() >= Center[2].point.x())
        {
            Center[2].point = sample.point; // 更新第一个中心点
            Center[2].diameters = sample.diameters; // 更新直径
        }
        mid_point = (Center[0].point.x() + Center[2].point.x()) / 2.0; // 计算中点(x轴)
        Center[1].point = AbsoluteDistance(sample.point.x(), mid_point) <= AbsoluteDistance(Center[1].point.x() ,mid_point) ? sample.point : Center[1].point; // 更新第二个中心点
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
    // 第一步：分配每个点到最近的簇
    for (auto& sample: samples)
    {
        double min_dist = distance(sample, Clustering_Result[0]);
        sample.cluster_id = 0;
        for(size_t j = 1; j < Clustering_Result.size(); ++j)
        {
            double dist = distance(sample, Clustering_Result[j]);
            if (min_dist > dist)
            {
                min_dist = dist;
                sample.cluster_id = j;
            }
        }
    }
    // 第二步：更新中心点
    Clustering_Result = calculate_center(samples);

    return Clustering_Result;
}