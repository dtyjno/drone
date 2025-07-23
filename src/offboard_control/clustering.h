#pragma once
#include <vector>
#include "OffboardControl.h"

struct Circles 
{
    Vector3d point;
    int cluster_id;
    double diameters; // 直径属性
};

// 声明全局变量
extern std::vector<Circles> Target_Samples;

double distance(Vector3d a, Vector3d b);
std::vector<Vector3d> calculate_center(std::vector<Circles> samples);
std::vector<Circles> Initialize_Clustering(std::vector<Circles> samples);
std::vector<Circles> Clustering(std::vector<Circles> samples);