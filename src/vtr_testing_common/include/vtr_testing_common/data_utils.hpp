// Utils for data accessing

#pragma once

#include <filesystem>
#include <fstream>
#include <sstream>
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
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  uint float_offset = 4;
  uint fields = 7;  // x, y, z, i, t, r, a
  uint point_step = float_offset * fields;
  uint N = floor(buffer.size() / point_step);
  Eigen::MatrixXd pc(Eigen::MatrixXd::Ones(N, fields));
  for (uint i = 0; i < N; ++i) {
    uint bufpos = i * point_step;
    for (uint j = 0; j < fields; ++j) {
      pc(i, j) =
          getFloatFromByteArray(buffer.data(), bufpos + j * float_offset);
    }
  }
  // Add offset to timestamps
  const auto timestamp = getStampFromPath(path);
  double t = double(timestamp / 1000) * 1.0e-6;
  pc.block(0, 4, N, 1).array() += t;

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
    CLOG(ERROR, "boreas_wrapper") << "IMU measurement file does not exist: " << imu_meas_file;
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