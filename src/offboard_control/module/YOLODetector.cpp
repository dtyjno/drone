#include <unordered_map>
#include "YOLODetector.h"

const std::unordered_map<YOLO_TARGET_TYPE, std::string> YOLODetector::target_type_strings = {
    {YOLO_TARGET_TYPE::CIRCLE, "circle"},
    {YOLO_TARGET_TYPE::H, "h"},
    {YOLO_TARGET_TYPE::STUFFED, "stuffed"}
};

std::string YOLODetector::enumToString(YOLO_TARGET_TYPE t){
    auto it = target_type_strings.find(t);
    return (it != target_type_strings.end()) ? it->second : "未知";
}