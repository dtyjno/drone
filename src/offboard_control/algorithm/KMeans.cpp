#include "KMeans.h"
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

// 增量式均值和方差（Welford算法）
struct IncrementalStats {
    size_t n = 0;
    double mean = 0.0;
    double M2 = 0.0;

    void add(double x) {
        n++;
        double delta = x - mean;
        mean += delta / n;
        double delta2 = x - mean;
        M2 += delta * delta2;
    }
    double variance() const { return n > 1 ? M2 / (n - 1) : 0.0; }
    double stddev() const { return sqrt(variance()); }
};

// KMeans聚类实现
KMeans::KMeans(int k, int max_iter)
    : K(k), max_iterations(max_iter)
{}

void KMeans::fit(const std::vector<std::vector<double>>& data) {
    if (data.empty() || data[0].empty()) return;
    int n_samples = data.size();
    int n_features = data[0].size();

    // 随机初始化中心
    centroids.clear();
    for (int i = 0; i < K; ++i) {
        centroids.push_back(data[i % n_samples]);
    }

    std::vector<int> labels(n_samples, -1);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 1. 分配每个点到最近中心
        bool changed = false;
        for (int i = 0; i < n_samples; ++i) {
            double min_dist = std::numeric_limits<double>::max();
            int min_id = -1;
            for (int j = 0; j < K; ++j) {
                double dist = 0.0;
                for (int f = 0; f < n_features; ++f) {
                    double d = data[i][f] - centroids[j][f];
                    dist += d * d;
                }
                if (dist < min_dist) {
                    min_dist = dist;
                    min_id = j;
                }
            }
            if (labels[i] != min_id) {
                changed = true;
                labels[i] = min_id;
            }
        }
        if (!changed && iter > 0) break;

        // 2. 更新中心
        std::vector<std::vector<double>> new_centroids(K, std::vector<double>(n_features, 0.0));
        std::vector<int> counts(K, 0);
        for (int i = 0; i < n_samples; ++i) {
            int cid = labels[i];
            for (int f = 0; f < n_features; ++f) {
                new_centroids[cid][f] += data[i][f];
            }
            counts[cid]++;
        }
        for (int j = 0; j < K; ++j) {
            if (counts[j] > 0) {
                for (int f = 0; f < n_features; ++f) {
                    new_centroids[j][f] /= counts[j];
                }
            }
        }
        centroids = new_centroids;
    }
    this->labels = labels;
}

const std::vector<std::vector<double>>& KMeans::getCentroids() const {
    return centroids;
}

const std::vector<int>& KMeans::getLabels() const {
    return labels;
}