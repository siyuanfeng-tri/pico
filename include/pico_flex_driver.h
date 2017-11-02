#pragma once

#include <boost/make_shared.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <royale/CameraManager.hpp>
#include <royale/String.hpp>
#include <royale/Vector.hpp>
#include <thread>
#include <utility>

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pico_flex_driver {

class PicoFlexDriver {
public:
  PicoFlexDriver() {}

  void StartCapture();
  void StopCapture();

  boost::shared_ptr<const cv::Mat> GetDepthImage(uint64_t *ctr) const;
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
  GetPointCloud(uint64_t *ctr) const;

private:
  class DepthDataHandler : public royale::IDepthDataListener {
  public:
    void onNewData(const royale::DepthData *data) override {
      auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      cloud->reserve(data->points.size());

      for (const auto &point : data->points) {
        // 0 = bad, 255 = good
        if (point.depthConfidence == 255) {
          pcl::PointXYZ pt{};
          pt.x = point.x;
          pt.y = point.y;
          pt.z = point.z;
          cloud->push_back(pt);
        }
      }

      std::lock_guard<std::mutex> guard(mutex_);
      cloud_ = cloud;
      ctr_++;
    }

    boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
    GetLatestPointCloud(uint64_t *ctr) const {
      std::lock_guard<std::mutex> guard(mutex_);
      *ctr = ctr_;
      return cloud_;
    }

  private:
    mutable std::mutex mutex_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_{nullptr};
    uint64_t ctr_{0};
  };

  class DepthImageHandler : public royale::IDepthImageListener {
  public:
    void onNewData(const royale::DepthImage *img) override {
      auto new_img =
          boost::make_shared<cv::Mat>(img->height, img->width, CV_16S);
      for (uint16_t i = 0; i < img->height; i++) {
        for (uint16_t j = 0; j < img->width; j++) {
          uint16_t data = img->data.at(i * img->width + j);
          // The least significant 13 bits are the depth (z value along the
          // optical axis) in millimeters. 0 stands for invalid measurement / no
          // data. The most significant 3 bits correspond to a confidence value.
          uint16_t z = data & 0x1fff;
          // uint16_t confidence = (data >> 13) & 0x7;
          new_img->at<uint16_t>(i, j) = z;
        }
      }

      std::lock_guard<std::mutex> guard(mutex_);
      depth_img_ = new_img;
      ctr_++;
    }

    boost::shared_ptr<const cv::Mat> GetLatestDepthImage(uint64_t *ctr) const {
      std::lock_guard<std::mutex> guard(mutex_);
      *ctr = ctr_;
      return depth_img_;
    }

  private:
    mutable std::mutex mutex_;
    boost::shared_ptr<cv::Mat> depth_img_{nullptr};
    uint64_t ctr_{0};
  };

  std::unique_ptr<royale::ICameraDevice> camera_;
  std::unique_ptr<DepthDataHandler> depth_data_handler_;
  std::unique_ptr<DepthImageHandler> depth_img_handler_;
};
}
