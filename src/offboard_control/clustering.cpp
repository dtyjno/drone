#include <iostream>
#include <vector>
#include <cmath>
#include "OffboardControl.h"
#include "clustering.h"

std::vector<Circles> Target_Samples;// 全局变量，存储目标样本点

// 计算均值
double computeMean(const std::vector<double>& data) 
{
    double sum = 0.0;
    for (double val : data) sum += val;
    return sum / data.size();
}

// 计算标准差
double computeStdDev(const std::vector<double>& data, double mean) 
{
    double sumSq = 0.0;
    for (double val : data) sumSq += (val - mean) * (val - mean);
    return std::sqrt(sumSq / data.size());
}

// 标准化处理
std::vector<Circles> normalizeCircles(const std::vector<Circles>& originalCircles) 
{
    std::vector<double> xs, ys, ds;
    for (const auto& c : originalCircles) 
    {
        xs.push_back(c.point.x());
        ys.push_back(c.point.y());
        ds.push_back(c.diameters);
    }

    double meanX = computeMean(xs);
    double meanY = computeMean(ys);
    double meanD = computeMean(ds);

    double stdX = computeStdDev(xs, meanX);
    double stdY = computeStdDev(ys, meanY);
    double stdD = computeStdDev(ds, meanD);
    std::vector<Circles> normalized;
    for (size_t i = 0; i < originalCircles.size(); ++i)
    {
        std::vector<Circles> c = originalCircles;
        Circles normC;
        normC.point.x() = (stdX != 0) ? (c[i].point.x() - meanX) / stdX : 0;
        normC.point.y() = (stdY != 0) ? (c[i].point.y() - meanY) / stdY : 0;
        normC.diameters = (stdD != 0) ? (c[i].diameters - meanD) / stdD : 0;
        normC.original_index = i; // 保留原始索引

        normalized.push_back(normC);
    }

    return normalized;
}

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
    
    return Center;
}

//分配每个数据点到最近的簇
void allocate_centers(std::vector<Circles> samples)
{
    std::vector<Circles> Clustering_Result = Initialize_Clustering(samples); // 清空之前的聚类结果
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
}

// 将标准化后的聚类结果映射回原始空间
vector<Circles> computeClusterCentersInOriginalSpace(
    const vector<Circles>& clusteredData,
    const vector<Circles>& originalData 
) {
    const int NUM_CLUSTERS = 3;

    vector<Vector2d> sumXY(NUM_CLUSTERS, Vector2d::Zero());
    vector<double> sumD(NUM_CLUSTERS, 0.0);
    vector<int> count(NUM_CLUSTERS, 0);

    // 遍历聚类结果，通过 original_index 找回原始数据
    for (const auto& c : clusteredData) 
    {
        int cid = c.cluster_id;
        int idx = c.original_index;

        const auto& orig = originalData[idx];
        sumXY[cid] += Vector2d(orig.point.x(), orig.point.y());
        sumD[cid] += orig.diameters;
        count[cid]++;
    }

    // 构造返回值
    vector<Circles> centers;
    for (int i = 0; i < NUM_CLUSTERS; ++i) 
    {
        if (count[i] == 0) continue;

        Circles center;
        center.point.x() = sumXY[i].x() / count[i];
        center.point.y() = sumXY[i].y() / count[i];
                                
        center.diameters = sumD[i] / count[i];
        center.cluster_id = i;
        center.original_index = -1; // 表示为聚类中心点

        centers.push_back(center);
    }

    return centers;
}

//聚类算法
std::vector<Circles> Clustering(std::vector<Circles> samples)
{
    std::vector<Circles> normalize_data = normalizeCircles(samples); // 标准化处理
    std::vector<Circles> Noramalize_Clustering = Initialize_Clustering(normalize_data); //初始化,Normallize_Clustering用于储存标准化后的聚类结果
    // 第一步：分配每个点到最近的簇
    allocate_centers(normalize_data);
    // 第二步：更新中心点
    Noramalize_Clustering = calculate_center(normalize_data);
    // 第三步：将标准化后的聚类结果映射回原始空间
    std::vector<Circles> Clustering_Result = computeClusterCentersInOriginalSpace(Noramalize_Clustering, samples);//Clustering_Result用于储存聚类结果

    return Clustering_Result;
}