#pragma once
#include <vector>
#include "utils/math.h"

#define MAX_DIAMETER 0.15
#define TRIM_DIAMETER 0.20
#define MIN_DIAMETER 0.25

struct Circles 
{
    Vector3d point;
    int cluster_id;
    size_t original_index; // 用于追踪原始数据点的索引
    double diameters; // 直径属性
};

struct ClusteringData {
    std::vector<Circles> sample;
    double sum_xs = 0, sum_ys = 0, sum_ds = 0;         // 用于计算均值
    double sumSq_xs = 0, sumSq_ys = 0, sumSq_ds = 0;   // 用于计算方差
    bool empty() const { return sample.empty(); }
};


double distance(Circles a, Circles b);
double computeMean(const std::vector<double>& data);
double computeStdDev(const std::vector<double>& data, double mean);
std::vector<Circles> normalizeCircles(const std::vector<Circles>& originalCircles);
std::vector<Circles> computeClusterCentersInOriginalSpace(const std::vector<Circles>& clusteredData,const std::vector<Circles>& originalData);
double AbsoluteDistance(double a, double b);
std::vector<Circles> calculate_center(const std::vector<Circles>& samples);
std::vector<Circles> Initialize_Clustering(const std::vector<Circles>& samples);
std::vector<Circles> Clustering(const std::vector<Circles>& samples);
std::vector<Circles> Clustering(ClusteringData& samples, const Circles& new_data);
