// Utils for data accessing

#pragma once

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdint>
#include <cstring>
#include <vector>

#include "vtr_testing_common/utils.hpp"
#include "vtr_testing_common/math_utils.hpp"

namespace fs = std::filesystem;

namespace vtr {
namespace testing {

inline void load_lidar_groundtruth(const fs::path &path, std::vector<lgmath::se3::Transformation> &all_gt_poses, std::vector<Eigen::Vector<double, 6>> &all_gt_vels) {
  std::ifstream ifs(path / "novatel" / "lidar_poses.csv", std::ios::in);
  // Clear header line
  std::string line;
  std::getline(ifs, line);
  // Loop through all gt data
  while (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::vector<double> gt;
    for (std::string str; std::getline(ss, str, ',');)
      gt.push_back(std::stod(str));

    // Store gt pose
    Eigen::Matrix4d T_ab_mat = Eigen::Matrix4d::Identity();
    T_ab_mat.block<3, 3>(0, 0) = rpy2rot(gt[4], gt[5], gt[6]);
    T_ab_mat.block<3, 1>(0, 3) << gt[1], gt[2], gt[3];
    lgmath::se3::Transformation T_ab = lgmath::se3::Transformation(T_ab_mat);
    all_gt_poses.push_back(T_ab.inverse());

    // Store gt velocity
    Eigen::Vector<double, 3> vbar;
    vbar << gt[13], gt[14], gt[15];
    vbar = T_ab_mat.block<3, 3>(0, 0).transpose() * vbar;
    Eigen::Vector<double, 6> body_rate;
    body_rate << vbar[0], vbar[1], vbar[2], gt[10], gt[11], gt[12];
    all_gt_vels.push_back(-body_rate);
  }
}

inline void load_radar_groundtruth(const fs::path &path, std::vector<lgmath::se3::Transformation> &all_gt_poses, std::vector<Eigen::Vector<double, 6>> &all_gt_vels) {
  std::ifstream ifs(path / "novatel" / "radar_poses.csv", std::ios::in);
  // Clear header line
  std::string line;
  std::getline(ifs, line);
  // Loop through all gt data
  while (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::vector<double> gt;
    for (std::string str; std::getline(ss, str, ',');)
      gt.push_back(std::stod(str));

    // Store gt pose
    Eigen::Matrix4d T_ab_mat = Eigen::Matrix4d::Identity();
    T_ab_mat.block<3, 3>(0, 0) = rpy2rot(roundToPi(gt[4]), roundToPi(gt[5]), gt[6]);
    T_ab_mat.block<3, 1>(0, 3) << gt[1], gt[2], 0.0;
    lgmath::se3::Transformation T_ab = lgmath::se3::Transformation(T_ab_mat);
    all_gt_poses.push_back(T_ab.inverse());

    // Store gt velocity
    Eigen::Vector<double, 3> vbar;
    vbar << gt[13], gt[14], gt[15];
    vbar = T_ab_mat.block<3, 3>(0, 0).transpose() * vbar;
    Eigen::Vector<double, 6> body_rate;
    body_rate << vbar[0], vbar[1], 0.0, 0.0, 0.0, gt[12];
    all_gt_vels.push_back(-body_rate);
  }
}

inline std::pair<int64_t, Eigen::MatrixXd> load_lidar(const std::string &path) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    CLOG(ERROR, "canoe_wrapper") << "Failed to open lidar file: " << path;
    throw std::runtime_error("Failed to open lidar file: " + path);
  }

  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  constexpr size_t fields = 7;  // x, y, z, i, t, r, a
  constexpr size_t point_step =
      sizeof(double) * 3 + sizeof(uint16_t) + sizeof(uint64_t) + sizeof(uint16_t) * 2;
  const size_t N = floor(buffer.size() / point_step);

  if (N == 0) {
    const auto timestamp = getStampFromPath(path);
    return std::make_pair(timestamp, Eigen::MatrixXd::Zero(0, fields));
  }

  Eigen::MatrixXd pc = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(N), fields);
  for (size_t i = 0; i < N; ++i) {
    const size_t bufpos = i * point_step;
    const char *p = buffer.data() + bufpos;

    double x = 0.0, y = 0.0, z = 0.0;
    uint16_t intensity = 0, reflectivity = 0, ambient = 0;
    uint64_t timestamp_us = 0;

    std::memcpy(&x, p, sizeof(double));
    std::memcpy(&y, p + sizeof(double), sizeof(double));
    std::memcpy(&z, p + sizeof(double) * 2, sizeof(double));
    std::memcpy(&intensity, p + sizeof(double) * 3, sizeof(uint16_t));
    std::memcpy(&timestamp_us,
                p + sizeof(double) * 3 + sizeof(uint16_t),
                sizeof(uint64_t));
    std::memcpy(&reflectivity,
                p + sizeof(double) * 3 + sizeof(uint16_t) + sizeof(uint64_t),
                sizeof(uint16_t));
    std::memcpy(&ambient,
                p + sizeof(double) * 3 + sizeof(uint16_t) + sizeof(uint64_t) + sizeof(uint16_t),
                sizeof(uint16_t));

    pc(i, 0) = x;
    pc(i, 1) = y;
    pc(i, 2) = z;
    pc(i, 3) = static_cast<double>(intensity);
    pc(i, 4) = static_cast<double>(timestamp_us) * 1.0e-6;
    pc(i, 5) = static_cast<double>(reflectivity);
    pc(i, 6) = static_cast<double>(ambient);
  }

  // Frame timestamp still comes from the file name for ROS clocking.
  const auto timestamp = getStampFromPath(path);
  return std::make_pair(timestamp, std::move(pc));
}

struct IMUMeasurement {
  // custom to prevent precision loss
  int64_t timestamp_ns;
  long double angvel_x;
  long double angvel_y;
  long double angvel_z;
};

inline void load_all_imu_meas(const fs::path &imu_meas_file, std::vector<IMUMeasurement> &all_imu_meas) {
  // Confirm file exists
  if (!fs::exists(imu_meas_file)) {
    CLOG(ERROR, "canoe_wrapper") << "IMU measurement file does not exist: " << imu_meas_file;
    throw std::invalid_argument("File not found: " + imu_meas_file.string());
  }
  
  std::ifstream imu_stream(imu_meas_file, std::ios::in);

  // Get rid of header
  std::string header;
  std::getline(imu_stream, header);

  // Loop over all imu measurements
  std::string imu_meas;

  while (std::getline(imu_stream, imu_meas)) {
    std::stringstream ss(imu_meas);
    std::string token;

    // Load timestamp separately to preserve precision
    std::getline(ss, token, ',');  // token = timestamp string
    int64_t timestamp_ns = stringToNanoseconds(token);
    // Load rest of the IMU data
    std::vector<long double> imu;
    while (std::getline(ss, token, ',')) {
      imu.push_back(std::stod(token));
    }

    IMUMeasurement meas;
    meas.timestamp_ns = static_cast<int64_t>(timestamp_ns);
    meas.angvel_x = imu[0];
    meas.angvel_y = imu[1];
    meas.angvel_z = imu[2];

    all_imu_meas.push_back(meas);
  }
}

} // namespace testing
} // namespace vtr