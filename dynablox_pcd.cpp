/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Qingwen Zhang (https://kin-zhang.github.io/)
 * @date: 2023-05-02 13:07
 * @details: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map / Detection
 */

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "dynablox/dynablox.h"


std::string extractTimestampFromFilename(const std::string& filename) {
    std::size_t last_slash = filename.find_last_of("/\\");
    std::string base_name = filename.substr(last_slash + 1);
    std::size_t dot = base_name.find_last_of(".");
    return base_name.substr(0, dot); // Extract the timestamp part of the filename
}

int main(int argc, char** argv) {
  /* #region Initial  */
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(google::INFO);

  if (argc < 3) {
      LOG(ERROR) << "Usage: " << argv[0] << " [pcd_folder] [config_file]";
      return 1;
  }
  std::string pcd_parent = argv[1];  // we assume that rawmap is in pcd_parent;
  std::string config_file = argv[2];
  std::string current_timestamp = argv[3];

  int cnt = 1, run_max = 1;
  // check if the config_file exists
  if (!std::filesystem::exists(config_file)) {
    LOG(ERROR) << "Config file does not exist: " << config_file;
    return 1;
  }
  std::vector<std::string> filenames;
  for (const auto& entry : std::filesystem::directory_iterator(
           std::filesystem::path(pcd_parent) / "pcd")) {
    filenames.push_back(entry.path().string());
  }

  // sort the filenames
  std::sort(filenames.begin(), filenames.end());

  int current_index = -1;
  for (size_t i = 0; i < filenames.size(); ++i) {
      std::string file_timestamp = extractTimestampFromFilename(filenames[i]);
      if (file_timestamp == current_timestamp) {
          current_index = i;
          break;
      }
  }

  if (current_index == -1) {
    LOG(ERROR) << "Timestamp " << current_timestamp << " not found in the filenames.";
    return 1;
  }

  int start_index = 0, end_index = 0;
  if (current_index < 150) {
    start_index = 0;
    end_index   = 300;
  } else if (current_index > filenames.size() - 1 - 150) {
    start_index = filenames.size() - 1 - 300;
    end_index   = filenames.size() - 1;
  } else {
    start_index = current_index - 150;
    end_index   = current_index + 150;
  }

  std::vector<std::string> submap_filenames(filenames.begin() + start_index, filenames.begin() + end_index + 1);
  filenames = submap_filenames;
  std::cout << "filenames size: " << filenames.size() << std::endl;



  int total = filenames.size();
  run_max = total + 1;
  // if (argc > 3) {
  //   run_max = std::stoi(argv[3]);
  //   if (run_max == -1) {
  //     LOG(INFO) << "We will run all the frame in sequence, the total "
  //                  "number is: "
  //               << total;
  //     run_max = total + 1;
  //   }
  // }
  /* #endregion */

  dynablox::MapUpdater map_updater(config_file);
  map_updater.timing.start("Total");
  for (const auto& filename : filenames) {
    map_updater.timing[0].start("One Scan Cost");
    if (cnt > 1 && !map_updater.getCfg().verbose_) {
      std::ostringstream log_msg;
      log_msg << "(" << cnt << "/" << run_max << ") Processing: " << filename
              << " Time Cost: " << map_updater.timing[0].lastSeconds() << "s";
      std::string spaces(10, ' ');
      log_msg << spaces;
      // std::cout <<log_msg.str()<<std::endl;
      std::cout << "\r" << log_msg.str() << std::flush;
    }

    if (filename.substr(filename.size() - 4) != ".pcd") continue;

    pcl::PointCloud<PointType>::Ptr pcd(new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile<PointType>(filename, *pcd);
    map_updater.run(pcd);
    map_updater.timing[0].stop();
    cnt++;
    if (cnt > run_max) break;
  }
  map_updater.saveMap(pcd_parent, current_timestamp);
  map_updater.timing.stop();
  map_updater.timing.print("Dynablox " /*title*/, true /*color*/,
                           true /*bold*/);

  return 0;
}