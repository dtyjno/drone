#include <iostream>
#include <vector>
#include <cmath>
#include "clustering.h"

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

// 分配每个点到最近的中心点，返回labels
std::vector<int> assignLabels(const std::vector<Circles>& data, const std::vector<Circles>& centers) 
{
    std::vector<int> labels(data.size(), 0);
    for (size_t i = 0; i < data.size(); ++i) {
        double min_dist = distance(data[i], centers[0]);
        int min_id = 0;
        for (size_t j = 1; j < centers.size(); ++j) {
            double dist = distance(data[i], centers[j]);
            if (dist < min_dist) {
                min_dist = dist;
                min_id = j;
            }
        }
        labels[i] = min_id;
    }
    return labels;
}

// 根据labels重新计算中心点
std::vector<Circles> updateCenters(const std::vector<Circles>& data, const std::vector<int>& labels, int K) 
{
    std::vector<Eigen::Vector2d> sumXY(K, Eigen::Vector2d::Zero());
    std::vector<double> sumD(K, 0.0);
    std::vector<int> count(K, 0);
    std::vector<Circles> centers(K);
    for (size_t i = 0; i < data.size(); ++i) {
        int cid = labels[i];
        sumXY[cid] += Eigen::Vector2d(data[i].point.x(), data[i].point.y());
        sumD[cid] += data[i].diameters;
        count[cid]++;
    }
    for (int j = 0; j < K; ++j) {
        if (count[j] > 0) {
            centers[j].point.x() = sumXY[j].x() / count[j];
            centers[j].point.y() = sumXY[j].y() / count[j];
            centers[j].diameters = sumD[j] / count[j];
        } else {
            centers[j].point.x() = 0;
            centers[j].point.y() = 0;
            centers[j].diameters = 0;
        }
        centers[j].cluster_id = j;
        centers[j].original_index = -1;
    }
    return centers;
}

// 反标准化中心点
std::vector<Circles> denormalizeCenters(const std::vector<Circles>& centers, double meanX, double stdX, double meanY, double stdY, double meanD, double stdD) 
{
    std::vector<Circles> result;
    for (const auto& c : centers) {
        Circles cc = c;
        cc.point.x() = c.point.x() * stdX + meanX;
        cc.point.y() = c.point.y() * stdY + meanY;
        cc.diameters = c.diameters * stdD + meanD;
        result.push_back(cc);
    }
    return result;
}

// 排序函数
bool cmpByDiameter(const Circles& a, const Circles& b) 
{
    return a.diameters > b.diameters;
}

// 主聚类算法
std::vector<Circles> Clustering(std::vector<Circles> samples)
{
    std::vector<Circles> data = normalizeCircles(samples);//数据标准化
    const int K = 3;
    const int max_iter = 300;//最大迭代次数
    std::vector<Circles> centers = Initialize_Clustering(data);
    std::vector<int> labels(data.size(), 0);
    
    //根据lables分配中心点（即簇id）
    for (int iter = 0; iter < max_iter; ++iter) 
    {
        std::vector<int> new_labels = assignLabels(data, centers);
        if (new_labels == labels && iter > 0)
        {
            break;
        } 
        labels = new_labels;
        centers = updateCenters(data, labels, K);
    }
    // 将结果映射回原始空间
    std::vector<double> xs, ys, ds;
    for (const auto& c : samples)
    {
        xs.push_back(c.point.x());
        ys.push_back(c.point.y());
        ds.push_back(c.diameters);
    }
    double meanX = computeMean(xs), meanY = computeMean(ys), meanD = computeMean(ds);
    double stdX = computeStdDev(xs, meanX), stdY = computeStdDev(ys, meanY), stdD = computeStdDev(ds, meanD);
    std::vector<Circles> result = denormalizeCenters(centers, meanX, stdX, meanY, stdY, meanD, stdD);
    // std::sort(result.begin(), result.end(), cmpByDiameter);
    
    return result;
}