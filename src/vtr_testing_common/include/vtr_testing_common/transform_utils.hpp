#pragma once

#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

#include <lgmath/se3/TransformationWithCovariance.hpp>

namespace fs = std::filesystem;
using EdgeTransform = lgmath::se3::TransformationWithCovariance;

namespace vtr {
namespace testing {

inline EdgeTransform load_T_robot_radar(const fs::path &path) {
    std::ifstream ifs1(path / "calib" / "T_boat_lidar.txt", std::ios::in);
    std::ifstream ifs2(path / "calib" / "T_lidar_radar.txt", std::ios::in);

    if (!ifs1.is_open()) {
        CLOG(ERROR, "canoe_wrapper") << "Could not open file: " << path / "calib" / "T_boat_lidar.txt";
        throw std::invalid_argument("File not found: " + (path / "calib" / "T_boat_lidar.txt").string());
    }
    if (!ifs2.is_open()) {
        CLOG(ERROR, "canoe_wrapper") << "Could not open file: " << path / "calib" / "T_lidar_radar.txt";
        throw std::invalid_argument("File not found: " + (path / "calib" / "T_lidar_radar.txt").string());
    }

    Eigen::Matrix4d T_boat_lidar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs1 >> T_boat_lidar_mat(row, col);

    Eigen::Matrix4d T_lidar_radar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs2 >> T_lidar_radar_mat(row, col);

    // Extrinsic from radar to boat
    EdgeTransform T_robot_radar(
        Eigen::Matrix4d(T_boat_lidar_mat * T_lidar_radar_mat),
        Eigen::Matrix<double, 6, 6>::Zero()
    );

    return T_robot_radar;
}

inline EdgeTransform load_T_robot_lidar(const fs::path &path) {
    std::ifstream ifs(path / "calib" / "T_boat_lidar.txt", std::ios::in);
    if (!ifs.is_open()) {
        CLOG(ERROR, "canoe_wrapper") << "Could not open file: " << path / "calib" / "T_boat_lidar.txt";
        throw std::invalid_argument("File not found: " + (path / "calib" / "T_boat_lidar.txt").string());
    }
    Eigen::Matrix4d T_boat_lidar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs >> T_boat_lidar_mat(row, col);

    // Extrinsic from lidar to boat
    EdgeTransform T_robot_lidar(
        Eigen::Matrix4d(T_boat_lidar_mat),
        Eigen::Matrix<double, 6, 6>::Zero()
    );

    return T_robot_lidar;
}

inline EdgeTransform load_T_robot_imu(const fs::path &path) {
    std::ifstream ifs1(path / "calib" / "T_boat_lidar.txt", std::ios::in);
    std::ifstream ifs2(path / "calib" / "T_lidar_imu.txt", std::ios::in);

    if (!ifs1.is_open()) {
        CLOG(ERROR, "canoe_wrapper") << "Could not open file: " << path / "calib" / "T_boat_lidar.txt";
        throw std::invalid_argument("File not found: " + (path / "calib" / "T_boat_lidar.txt").string());
    }
    if (!ifs2.is_open()) {
        CLOG(ERROR, "canoe_wrapper") << "Could not open file: " << path / "calib" / "T_lidar_imu.txt";
        throw std::invalid_argument("File not found: " + (path / "calib" / "T_lidar_imu.txt").string());
    }

    Eigen::Matrix4d T_boat_lidar_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs1 >> T_boat_lidar_mat(row, col);

    Eigen::Matrix4d T_lidar_imu_mat;
    for (size_t row = 0; row < 4; row++)
        for (size_t col = 0; col < 4; col++) ifs2 >> T_lidar_imu_mat(row, col);

    // Extrinsic from imu to boat
    EdgeTransform T_robot_imu(
        Eigen::Matrix4d(T_boat_lidar_mat * T_lidar_imu_mat),
        Eigen::Matrix<double, 6, 6>::Zero()
    );

    return T_robot_imu;
}


inline EdgeTransform load_T_enu_init(const fs::path &path, const bool &reverse, const std::string &sensor="lidar") {
    std::ifstream ifs(path / "novatel" / (sensor + "_poses.csv"), std::ios::in);

    if (!ifs.is_open()) {
        CLOG(ERROR, "canoe_wrapper") << "Could not open file: " << path / "novatel" / (sensor + "_poses.csv");
        throw std::invalid_argument("File not found: " + (path / "novatel" / (sensor + "_poses.csv")).string());
    }

    std::string header;
    std::getline(ifs, header);

    std::string first_pose;
    if (reverse) {
        std::string last_pose;
        // If reverse, we want to get last line
        while (std::getline(ifs, last_pose)) {
            first_pose = last_pose;
        }
    } else {
        std::getline(ifs, first_pose);
    }

    std::stringstream ss{first_pose};
    std::vector<double> gt;
    for (std::string str; std::getline(ss, str, ',');)
        gt.push_back(std::stod(str));

    Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
    T_mat.block<3, 3>(0, 0) = rpy2rot(gt[4], gt[5], gt[6]);
    T_mat.block<3, 1>(0, 3) << gt[1], gt[2], gt[3];

    EdgeTransform T(T_mat);
    T.setZeroCovariance();

    return T;
}

} // namespace testing
} // namespace vtr