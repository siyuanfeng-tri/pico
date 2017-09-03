#pragma once

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

namespace pico_flex_driver {

class DepthDataHandler : public royale::IDepthDataListener {
public:
  void onNewData(const royale::DepthData *data) override {
    std::lock_guard<std::mutex> guard(mutex_);
    depth_data_ = *data;
    ctr_++;
  }

  royale::DepthData CopyLatestDepthData(uint64_t *ctr) const {
    std::lock_guard<std::mutex> guard(mutex_);
    *ctr = ctr_;
    return depth_data_;
  }

private:
  mutable std::mutex mutex_;
  royale::DepthData depth_data_{};
  uint64_t ctr_{0};
};

class DepthImageHandler : public royale::IDepthImageListener {
public:
  void onNewData(const royale::DepthImage *img) override {
    std::lock_guard<std::mutex> guard(mutex_);
    depth_img_ = *img;
    ctr_++;
  }

  royale::DepthImage CopyLatestDepthImage(uint64_t *ctr) const {
    std::lock_guard<std::mutex> guard(mutex_);
    *ctr = ctr_;
    return depth_img_;
  }

private:
  mutable std::mutex mutex_;
  royale::DepthImage depth_img_{};
  uint64_t ctr_{0};
};

class PicoFlexDriver {
public:
  struct DepthImage {
    // This is CV_32F, in [mm]
    cv::Mat image;
    uint64_t timestamp;
    uint64_t id;
  };

  PicoFlexDriver() {
    depth_data_handler_.reset(new DepthDataHandler);
    depth_img_handler_.reset(new DepthImageHandler);
  }

  void StartCapture();
  void StopCapture();

  DepthImage GetDepthImage() const;

private:
  std::unique_ptr<royale::ICameraDevice> camera_;
  std::unique_ptr<DepthDataHandler> depth_data_handler_;
  std::unique_ptr<DepthImageHandler> depth_img_handler_;
};
}
