#ifndef UTILS_H
#define UTILS_H

#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include "Timer.h"

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
  const T cs = std::cos(angle);
  const T sn = std::sin(angle);
  T rx = x * cs - y * sn;
  T ry = x * sn + y * cs;
  x = rx;
  y = ry;
}

#endif // UTILS_H
