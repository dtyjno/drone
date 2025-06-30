#ifndef UTILS_H
#define UTILS_H

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>

// 模拟 _kbhit() 功能
inline int _kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt); // 获取终端属性
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 设置非规范模式，不回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 恢复终端属性
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin); // 将字符放回输入流
        return 1;
    }

    return 0;
}

// 模拟 _getch() 功能
inline char _getch() {
    struct termios oldt, newt;
    char ch;

    tcgetattr(STDIN_FILENO, &oldt); // 获取终端属性
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 设置非规范模式，不回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 恢复终端属性
    return ch;
}

// 顺时针旋转 2D 坐标点 (x, y) 以原点为中心，旋转角度为 angle（弧度制）
template <typename T>
void rotate_angle(T &x,T &y, float angle) {
  const T cs = cos(angle);
  const T sn = sin(angle);
  T rx = x * cs - y * sn;
  T ry = x * sn + y * cs;
  x = rx;
  y = ry;
}


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
    double elapsed() {
        if (start_time_ == std::chrono::steady_clock::time_point()) {
            // 如果 start_time_ 是默认时间点，返回 0
            start_time_ = std::chrono::steady_clock::now(); // 更新为当前时间
            return 0.0;
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
private:
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point time_point_;
};


#endif // UTILS_H
