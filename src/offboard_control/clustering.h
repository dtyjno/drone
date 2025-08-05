#pragma once
#include <vector>
#include "OffboardControl.h"

#define MAX_DIAMETER 0.15
#define TRIM_DIAMETER 0.20
#define MIN_DIAMETER 0.25

struct Circles 
{
    Vector3d point;
    int cluster_id;
    double diameters; // 直径属性
};

// 声明全局变量
extern std::vector<Circles> Target_Samples;

double distance(Circles a, Circles b);
double AbsoluteDistance(double a, double b);
std::vector<Circles> calculate_center(std::vector<Circles> samples);
std::vector<Circles> Initialize_Clustering(std::vector<Circles> samples);
std::vector<Circles> Clustering(std::vector<Circles> samples);
