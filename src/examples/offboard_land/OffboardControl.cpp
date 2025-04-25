#include "OffboardControl.h"
#include <iostream>
#include <ncurses.h>
#include <unistd.h>

void OffboardControl::timer_callback(void){
	_motors->command_takeoff_or_land("LAND");


	// if(first){
	// 	initscr();            // 初始化 ncurses
	// 	cbreak();             // 禁用行缓冲
	// 	noecho();             // 禁止输入回显
	// 	nodelay(stdscr, TRUE); // 非阻塞输入
	// 	keypad(stdscr, TRUE); // 启用功能键
	// 	first = false;
	// }
	// ch = getch();
	// if (ch != ERR) {
	// 	switch (ch) {
  //       case 'a':
	// 		_motors->arm_motors(false);
	// 		std::cout << "DISARM" << std::endl;
  //           break;
	// 	case 'l':
	// 		_motors->command_takeoff_or_land("LAND");
	// 		std::cout << "LAND" << std::endl;
  //           break;
	// 	// case 'w':
	// 	// 	_motors->command_takeoff_or_land("LAND",Vector4f{InertialNav::gps.x+1,InertialNav::gps.y,0,0});
	// 	// 	std::cout << "LAND" << std::endl;
  //       //     break;
	// 	// case 'a':
	// 	// 	_motors->command_takeoff_or_land("LAND",Vector4f{InertialNav::gps.x,InertialNav::gps.y+1,0,0});
	// 	// 	std::cout << "LAND" << std::endl;
  //       //     break;
	// 	// case 's':
	// 	// 	_motors->command_takeoff_or_land("LAND",Vector4f{InertialNav::gps.x-1,InertialNav::gps.y,0,0});
	// 	// 	std::cout << "LAND" << std::endl;
  //       //     break;
	// 	// case 'd':
	// 	// 	_motors->command_takeoff_or_land("LAND",Vector4f{InertialNav::gps.x,InertialNav::gps.y-1,0,0});
	// 	// 	std::cout << "LAND" << std::endl;
  //       //     break;
  //       default:
  //           break;
	// 	}
	// }
	//RCLCPP_INFO(this->get_logger(),"--------timer_callback----------");
	
	// std::cout << get_x_pos() <<" yaw: "<< get_n_yaw() <<" "<<get_armed()<< std::endl;
	// RCLCPP_INFO(this->get_logger(),"cur_time: %f %f, start_time: %f",get_cur_time(),(this->get_clock()->now().nanoseconds() / 1000- timestamp_init)/1000000.0,timestamp_init);
}