#pragma once

#include <boost/make_shared.hpp>
#include <iostream>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <royale/ICameraDevice.hpp>
#include <royale/IDepthDataListener.hpp>
#include <royale/IDepthImageListener.hpp>

namespace pico_flex_driver {

class PicoFlexDriver {
public:
  PicoFlexDriver() {}

  void StartCapture();
  void StopCapture();

  /**
   * Returns a shared pointer of a const cv::Mat that contains the latest
   * depth image or nullptr if no valid image has arrived yet. The caller is
   * responsible for making a deep copy of the returned image if necessary.
   * @param timestamp Timestamp of the depth image, in milliseconds.
   */
  boost::shared_ptr<const cv::Mat> GetDepthImage(uint64_t *timestamp) const;

  /**
   * Returns a shared pointer of a const point cloud or nullptr if no valid
   * cloud data has arrived yet. The caller is responsible for making a deep
   * copy of the returned cloud if necessary.
   * @param timestamp Timestamp of the point cloud, in milliseconds.
   */
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
  GetPointCloud(uint64_t *timestamp) const;

private:
  class DepthDataHandler : public royale::IDepthDataListener {
  public:
    void onNewData(const royale::DepthData *data) override {
      auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      cloud->reserve(data->points.size());

      for (const auto &point : data->points) {
        // TODO you can try to filter it by noise.

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
      timestamp_ = data->timeStamp.count();
    }

    boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
    GetLatestPointCloud(uint64_t *timestamp) const {
      std::lock_guard<std::mutex> guard(mutex_);
      *timestamp = timestamp_;
      return cloud_;
    }

  private:
    mutable std::mutex mutex_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_{nullptr};
    uint64_t timestamp_{0};
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
      timestamp_ = static_cast<uint64_t>(img->timestamp);
    }

    boost::shared_ptr<const cv::Mat>
    GetLatestDepthImage(uint64_t *timestamp) const {
      std::lock_guard<std::mutex> guard(mutex_);
      *timestamp = timestamp_;
      return depth_img_;
    }

  private:
    mutable std::mutex mutex_;
    boost::shared_ptr<cv::Mat> depth_img_{nullptr};
    uint64_t timestamp_{0};
  };

  std::unique_ptr<royale::ICameraDevice> camera_{nullptr};
  std::unique_ptr<DepthDataHandler> depth_data_handler_{nullptr};
  std::unique_ptr<DepthImageHandler> depth_img_handler_{nullptr};
};
}
