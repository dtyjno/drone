#pragma once

#include <opencv2/opencv.hpp>

struct CameraDataStruct {
    cv::Mat image; // 图像数据
    double timestamp; // 时间戳
    // 其他相机相关数据
};

// 观察者接口，用于接收相机数据更新
class CameraDataObserverInterface {
public:
    virtual ~CameraDataObserverInterface() = default;
    virtual void on_image_update(const CameraDataStruct& image) = 0;
};

class CameraDataObservable {
public:
    std::vector<CameraDataObserverInterface*> observers;
    void add_observer(CameraDataObserverInterface* observer) {
        observers.push_back(observer);
    }

    void remove_observer(CameraDataObserverInterface* observer) {
        observers.erase(std::remove(observers.begin(), observers.end(), observer), observers.end());
    }
protected:
    void notify_image_update(const CameraDataStruct& image) {
        for (auto observer : observers) {
            observer->on_image_update(image);
        }
    }
};