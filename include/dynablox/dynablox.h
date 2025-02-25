/**
 * Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author Qingwen Zhang (https://kin-zhang.github.io/)
 * @date: 2023-05-02 13:19
 * @details: No ROS version, speed up the process, check our benchmark in dufomap
 */
#pragma once

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <voxblox/core/block.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <yaml-cpp/yaml.h>

#include "dynablox/clustering.h"
#include "dynablox/types.h"

#include "common/TsdfMapper.h"
#include "common/timing.hpp"
#include "common/utils.h"

namespace dynablox {

class MapUpdater {
 public:
  MapUpdater(const std::string& config_file_path);
  virtual ~MapUpdater() = default;

  ufo::Timing timing;

  void setConfig();
  void run(pcl::PointCloud<PointType>::Ptr const& single_pc, std::string const& file_path);
  void saveMap(std::string const& folder_path, std::string const& file_name);
  const common::Config getCfg() { return config_; }

 private:
  common::Config config_;
  YAML::Node yconfig;

  size_t frame_counter_ = 0;

  std::shared_ptr<voxblox::TsdfMapper> tsdf_mapper_;
  std::shared_ptr<TsdfLayer> tsdf_layer_;

  std::shared_ptr<Clustering> clustering_;
  std::shared_ptr<EverFreeIntegrator> ever_free_integrator_;

  bool processPointCloud(pcl::PointCloud<PointType>& cloud, CloudInfo& cloud_info);
  void Tracking(const Cloud& cloud, Clusters& clusters, CloudInfo& cloud_info);
  void setUpPointMap(const pcl::PointCloud<PointType>& cloud, BlockToPointMap& point_map,
                     std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices, CloudInfo& cloud_info) const;
  void trackClusterIDs(const Cloud& cloud, Clusters& clusters);

  voxblox::HierarchicalIndexIntMap buildBlockToPointsMap(const pcl::PointCloud<PointType>& cloud) const;
  void blockwiseBuildPointMap(const pcl::PointCloud<PointType>& cloud, const BlockIndex& block_index,
                              const voxblox::AlignedVector<size_t>& points_in_block, VoxelToPointMap& voxel_map,
                              std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
                              CloudInfo& cloud_info) const;

  // Tracking data w.r.t. previous observation.
  std::vector<voxblox::Point> previous_centroids_;
  std::vector<int> previous_ids_;
  std::vector<int> previous_track_lengths_;
  pcl::PointCloud<PointType>::Ptr Dynamic_Cloud_, Static_Cloud_;
};
}  // namespace dynablox