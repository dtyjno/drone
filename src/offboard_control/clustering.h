#include <iostream>
#include <vector>
#include <cmath>
#include "OffboardControl.h"

#pragma once

using namespace std;
//定义二维平面点
struct Point
{
    double x,y;
    int cluster_id;
};

Point calculate_center(const vector<Point>& points, int cluster_id);
double distance(const Point& a, const Point& b);
std::optional<Vector3d> Clustering(vector<std::optional<Vector3d>> samples);
//样本点 
vector<std::optional<Vector3d>> samples;