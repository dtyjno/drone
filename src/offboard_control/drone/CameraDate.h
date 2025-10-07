#pragma once

#include <opencv2/opencv.hpp>
#include "CameraDataObserverInterface.h"

class CameraData : public CameraDataObservable {
public:
    CameraData() : cur_camera_data() {}

    CameraDataStruct cur_camera_data;
};