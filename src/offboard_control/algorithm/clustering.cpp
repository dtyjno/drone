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
std::vector<Circles> normalizeCircles(const std::vector<Circles>& originalCircles, 
                                      double meanX, double stdX, 
                                      double meanY, double stdY, 
                                      double meanD, double stdD) 
{
    std::vector<Circles> normalized;
    normalized.reserve(originalCircles.size());
    for (size_t i = 0; i < originalCircles.size(); ++i)
    {
        Circles normC;
        normC.point.x() = (stdX != 0) ? (originalCircles[i].point.x() - meanX) / stdX : 0;
        normC.point.y() = (stdY != 0) ? (originalCircles[i].point.y() - meanY) / stdY : 0;
        normC.diameters = (stdD != 0) ? (originalCircles[i].diameters - meanD) / stdD : 0;
        normC.original_index = i;
        normalized.emplace_back(normC);
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
std::vector<Circles> Initialize_Clustering(const std::vector<Circles>& samples)
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
    result.reserve(centers.size());
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
std::vector<Circles> Clustering(const std::vector<Circles>& samples)
{
    // 只计算一次均值和方差
    std::vector<double> xs, ys, ds;
    xs.reserve(samples.size());
    ys.reserve(samples.size());
    ds.reserve(samples.size());
    for (const auto& c : samples)
    {
        xs.push_back(c.point.x());
        ys.push_back(c.point.y());
        ds.push_back(c.diameters);
    }
    double meanX = computeMean(xs), meanY = computeMean(ys), meanD = computeMean(ds);
    double stdX = computeStdDev(xs, meanX), stdY = computeStdDev(ys, meanY), stdD = computeStdDev(ds, meanD);

    std::vector<Circles> data = normalizeCircles(samples, meanX, stdX, meanY, stdY, meanD, stdD);
    const int K = 3;
    const int max_iter = 100; // 可适当减小
    std::vector<Circles> centers = Initialize_Clustering(data);
    std::vector<int> labels(data.size(), -1);

    for (int iter = 0; iter < max_iter; ++iter) 
    {
        std::vector<int> new_labels = assignLabels(data, centers);
        if (new_labels == labels && iter > 0)
            break;
        labels = new_labels;
        centers = updateCenters(data, labels, K);
    }
    std::vector<Circles> result = denormalizeCenters(centers, meanX, stdX, meanY, stdY, meanD, stdD);
    return result;
}


// 增量式更新统计量
void updateClusteringData(ClusteringData& data, const Circles& new_circle) {
    data.sample.push_back(new_circle);
    data.sum_xs += new_circle.point.x();
    data.sum_ys += new_circle.point.y();
    data.sum_ds += new_circle.diameters;
    data.sumSq_xs += new_circle.point.x() * new_circle.point.x();
    data.sumSq_ys += new_circle.point.y() * new_circle.point.y();
    data.sumSq_ds += new_circle.diameters * new_circle.diameters;
}

// 增量式均值和方差
void computeMeanStd(const ClusteringData& data, double& meanX, double& stdX, double& meanY, double& stdY, double& meanD, double& stdD) {
    size_t n = data.sample.size();
    meanX = data.sum_xs / n;
    meanY = data.sum_ys / n;
    meanD = data.sum_ds / n;
    stdX = std::sqrt((data.sumSq_xs - n * meanX * meanX) / n);
    stdY = std::sqrt((data.sumSq_ys - n * meanY * meanY) / n);
    stdD = std::sqrt((data.sumSq_ds - n * meanD * meanD) / n);
}

// 增量式聚类主入口
std::vector<Circles> Clustering(ClusteringData& samples, const Circles& new_data) {
    updateClusteringData(samples, new_data);
    double meanX, stdX, meanY, stdY, meanD, stdD;
    computeMeanStd(samples, meanX, stdX, meanY, stdY, meanD, stdD);
    // 标准化处理
    std::vector<Circles> data = normalizeCircles(samples.sample, meanX, stdX, meanY, stdY, meanD, stdD);
    const int K = 3;
    const int max_iter = 100;
    std::vector<Circles> centers = Initialize_Clustering(data);
    std::vector<int> labels(data.size(), -1);

    for (int iter = 0; iter < max_iter; ++iter) {
        std::vector<int> new_labels = assignLabels(data, centers);
        if (new_labels == labels && iter > 0)
            break;
        labels = new_labels;
        centers = updateCenters(data, labels, K);
    }
    std::vector<Circles> result = denormalizeCenters(centers, meanX, stdX, meanY, stdY, meanD, stdD);
    return result;
}