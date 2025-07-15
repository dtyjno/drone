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
vector<Vector3d> Initialize_Clustering(vector<Points> samples)
{
    vector<Points> Center(3);
    Center[0] = samples[0];
    Center[2] = samples[samples.size() - 1];
    Center[1].point = (Center[0].point + Center[2].point) / 2;
    for(size_t i = 0; i < samples.size(); i++)
    {
        Center[0].point = Center[0].point.x() > samples[i].point.x()?samples[i].point:Center[0].point;
        Center[2].point = Center[2].point.x() < samples[i].point.x()?samples[i].point:Center[2].point;
        Center[1].point = (Center[0].point + Center[2].point) / 2;
    }
    vector<Vector3d> I_Center(3);
    I_Center[0] = Center[0].point;
    I_Center[1] = Center[1].point;
    I_Center[2] = Center[2].point;
    return I_Center;
}

//聚类算法
vector<Vector3d> Clustering(vector<Points> samples)
{
    vector<Vector3d> Clustering_Result = Initialize_Clustering(samples);
    // 第一步：分配每个点到最近的簇
    for (size_t i = 0; i < samples.size(); i++ )
    {
        double min_dist = distance(samples[i].point, Clustering_Result[0]);
        samples[i].cluster_id = 0;
        for(size_t j = 1; j < Clustering_Result.size(); j++)
        {
            double dist = distance(samples[i].point, Clustering_Result[j]);
            if (min_dist > dist)
            {
                min_dist = dist;
                samples[i].cluster_id = j;
            }
        }
    }
    // 第二步：更新中心点
    Clustering_Result = calculate_center(samples);
    return Clustering_Result;
}