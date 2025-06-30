#ifndef READYAML_H
#define READYAML_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>  
#include <iostream>
#include <filesystem>  // C++17 文件系统库

class Readyaml {
public:
    static const std::string& getPackageName() {
        static const std::string package_name = "px4_ros_com";
        return package_name;
    }

    static YAML::Node readYAML(const std::string& filename) {
        // 获取包的共享目录
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(getPackageName());
        // 构建相对配置文件路径
        std::string relative_path = package_share_directory + "/../../../../src/" + getPackageName() + "/config/" + filename;
        // 转换为绝对路径并规范化
        std::filesystem::path config_file_path = std::filesystem::canonical(relative_path);
        // std::cout << "config_file_path: " << config_file_path << std::endl;

        // 检查文件是否存在
        if (!std::filesystem::exists(config_file_path)) {
            throw std::runtime_error("Config file not found: " + config_file_path.string());
        }

        // 检查文件权限
        std::filesystem::perms p = std::filesystem::status(config_file_path).permissions();
        if ((p & std::filesystem::perms::owner_read) == std::filesystem::perms::none) {
            throw std::runtime_error("Config file is not readable: " + config_file_path.string());
        }

        try {
            // 读取配置文件
            // std::cout << "Loading config file: " << config_file_path.string() << std::endl;
            YAML::Node config = YAML::LoadFile(config_file_path.string());
            return config;
        } catch (const YAML::BadFile& e) {
            std::cerr << "YAML::BadFile exception: " << e.what() << std::endl;
            throw;
        } catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
            throw;
        }
    }
};

#endif // READYAML_H