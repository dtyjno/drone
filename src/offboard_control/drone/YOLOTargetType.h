#pragma once

    enum YOLO_TARGET_TYPE{
        CIRCLE, //0
        H, //+=1
        STUFFED
    };
    class TargetData
    {
    public:
        TargetData() = default; // 默认构造函数
        TargetData(float x, float y, float z, float r, float g, float b, float radius, const std::string &category, int id)
            : x(x), y(y), z(z), r(r), g(g), b(b), radius(radius), category(category), id(id) {}
        float x, y, z;           // 圆心坐标
        float r = 1, g = 0, b = 0;           // 颜色
        float radius;           // 半径
        std::string category;   // 分类标签
        int id;                 // 目标ID
        float fx = 1;
        float relative_z = 1; // 相对高度
        float caculate_pixel_radius(void) const {
            if (relative_z <= 0.01f) { // 防止除零
                return 5.0f; // 返回最小半径
            }
            float pixel_radius = (radius / relative_z) * fx; // 使用fx作为代表焦距
            return std::max(pixel_radius, 5.0f); // 最小半径为5像素
        }
    };