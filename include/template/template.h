#ifndef VOXBLOX_ROS_SEGMENT_SERVER_H_
#define VOXBLOX_ROS_SEGMENT_SERVER_H_

#include <cloud_segmentation/cloud_segmentation.h>

#include "voxblox_ros/esdf_server.h"
#include "voxblox_ros/ros_params.h"

    namespace voxblox {
  struct SIDVoxel {
    uint16_t sid;
  };

  class SegmentMap {
   public:
    struct Config {};

    explicit SegmentMap(const Config& config,
                        const TsdfMap::Config& tsdf_config)
        : config_(config),
          sid_layer_(new Layer<SIDVoxel>(tsdf_config.tsdf_voxel_size,
                                         tsdf_config.tsdf_voxels_per_side)) {}
    ~SegmentMap() = default;

    bool addSegment(const Pointcloud& segment_cloud) {
      for (auto const& point : segment_cloud) {

      }
      return true;
    }

    Config config_;
    std::unique_ptr<Layer<SIDVoxel>> sid_layer_;
  };

  class SegmentServer : public EsdfServer {
   public:
    struct Config {};

    static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);
    SegmentServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
        : EsdfServer(nh, nh_private),
          cloud_segmentation_(nh, nh_private),
          segment_map_(new SegmentMap(
              SegmentMap::Config(), getTsdfMapConfigFromRosParam(nh_private))) {
      CHECK(!tsdf_integrator_->getConfig().voxel_carving_enabled)
          << "Segment map can't have voxel carving enabled (for now)";
    }

    ~SegmentServer() = default;

    void updateEsdf() override {
      EsdfServer::updateEsdf();

      // Update segments following esdf updating
      updateSegments();
    }

    void updateSegments() {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::Normal>::Ptr normals(
          new pcl::PointCloud<pcl::Normal>);
      const bool only_updated_blocks = true;
      createDirectionPointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                             only_updated_blocks, points.get(),
                                             normals.get());

      if (cloud_segmentation.processPCLPointCloud(points)) {
        auto const& clusters = cloud_segmentation.getClusters();
        for (auto point_indices : clusters) {
          Pointcloud cluster_points;
          for (auto point_index : point_indices.indices) {
            Point point{(*points)[point_index].x, (*points)[point_index].y,
                        (*points)[point_index].z};
            cluster_points.emplace_back(point);
          }
          segment_map_->addSegment(cluster_points);
        }
      }
    }

    void updateSegmentsEvent(const ros::TimerEvent& /*event*/) {
      updateSegments();
    }

    Config config_;

    CloudSegmentation cloud_segmentation_;

    std::shared_ptr<SegmentMap> segment_map_;
  };
}  // namespace voxblox

#endif  // VOXBLOX_ROS_SEGMENT_SERVER_H_
