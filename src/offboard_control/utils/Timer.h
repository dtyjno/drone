#pragma once
#include <chrono>

class Timer {
public:
    Timer() 
        : start_time_(std::chrono::steady_clock::now()) {}

    /// @brief 无条件重置计时器
    void reset() {
        start_time_ = std::chrono::steady_clock::now();
    }

    /// @brief 获取自计时开始经过的时间（秒）
    double elapsed() const {
        if (start_time_ == std::chrono::steady_clock::time_point()) {
            // 如果 start_time_ 是默认时间点，返回最大值
            // start_time_ = std::chrono::steady_clock::now(); // 更新为当前时间
            return std::numeric_limits<double>::max();
        }
        const auto end_time = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(end_time - start_time_).count();
    }

    // 标记当前时间
    void set_timepoint(){
        time_point_ = std::chrono::steady_clock::now();
    }

    // 距上次标记经过时间（秒）
    double get_timepoint_elapsed(){
        const auto end_time = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(end_time - time_point_).count();
    }

    void set_start_time_to_default(){
        // 将计时器的开始时间设置为默认时间点
        start_time_ = std::chrono::steady_clock::time_point();
    }

    void set_start_time_to_time_point(double seconds_from_now) {
        // 将计时器的开始时间设置为从当前时间偏移指定秒数的时间点
        auto offset = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(seconds_from_now));
        start_time_ = std::chrono::steady_clock::now() - offset;
        }

    /// @brief 将start_time_设置为当前时间减去seconds秒
    void set_start_time_offset(double seconds) {
        auto offset = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(seconds));
        start_time_ = start_time_ - offset;
        time_point_ = time_point_ - offset;
    }
private:
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point time_point_;
};

