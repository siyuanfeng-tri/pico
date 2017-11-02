#include "pico_flex_driver.h"

namespace pico_flex_driver {

void PicoFlexDriver::StartCapture() {
  depth_data_handler_ = std::make_unique<DepthDataHandler>();
  depth_img_handler_ = std::make_unique<DepthImageHandler>();

  royale::CameraManager manager;
  auto camlist = manager.getConnectedCameraList();
  std::cout << "Detected " << camlist.size() << " camera(s)." << std::endl;
  if (!camlist.empty()) {
    std::cout << "CamID for first device: " << camlist.at(0).c_str()
              << " with a length of (" << camlist.at(0).length() << ")"
              << std::endl;
    camera_ = manager.createCamera(camlist[0]);
  }

  if (!camera_) {
    throw std::logic_error("Failed to open camera.");
  }

  if (camera_->initialize() != royale::CameraStatus::SUCCESS) {
    throw std::logic_error("Cannot initialize camera.");
  }

  royale::Vector<royale::String> use_cases;
  if (camera_->getUseCases(use_cases) != royale::CameraStatus::SUCCESS ||
      use_cases.empty()) {
    throw std::logic_error("Cannot find valid use cases.");
  }

  // Set mode.
  royale::String desired_mode{"MODE_5_45FPS_500"};
  bool has_mode = false;
  for (const auto &use_case : use_cases) {
    if (use_case == desired_mode) {
      has_mode = true;
      break;
    }
  }
  if (has_mode) {
    if (camera_->setUseCase(desired_mode) != royale::CameraStatus::SUCCESS) {
      throw std::logic_error("Cannot set mode: " + desired_mode.toStdString());
    }
  } else {
    throw std::logic_error("Does not have mode " + desired_mode.toStdString());
  }

  // Set camera exposure. (micro seconds)
  if (camera_->setExposureTime(10) != royale::CameraStatus::SUCCESS) {
    // Register callbacks.
    if (camera_->registerDataListener(depth_data_handler_.get()) !=
        royale::CameraStatus::SUCCESS) {
      throw std::logic_error("Error registering depth data listener");
    }
  }
  if (camera_->registerDepthImageListener(depth_img_handler_.get()) !=
      royale::CameraStatus::SUCCESS) {
    throw std::logic_error("Error registering depth image listener");
  }

  if (camera_->startCapture() != royale::CameraStatus::SUCCESS) {
    throw std::logic_error("Error starting the capturing.");
  }
}

void PicoFlexDriver::StopCapture() {
  if (camera_) {
    if (camera_->stopCapture() != royale::CameraStatus::SUCCESS) {
      throw std::logic_error("Error stopping the capturing.");
    }
    camera_.reset();
    depth_data_handler_.reset();
    depth_img_handler_.reset();
  }
}

boost::shared_ptr<const cv::Mat>
PicoFlexDriver::GetDepthImage(uint64_t *ctr) const {
  if (!camera_) {
    throw std::logic_error("Camera is not catpuring.");
  }

  return depth_img_handler_->GetLatestDepthImage(ctr);
}

boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>
PicoFlexDriver::GetPointCloud(uint64_t *ctr) const {
  if (!camera_) {
    throw std::logic_error("Camera is not catpuring.");
  }

  return depth_data_handler_->GetLatestPointCloud(ctr);
}
}
