#include "driver.h"

namespace pico_flex_driver {

void PicoFlexDriver::StartCapture() {
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
    throw std::logic_error("Does not have mode " +
                           desired_mode.toStdString());
  }

  // Set camera exposure. (micro seconds)
  if (camera_->setExposureTime(10) != royale::CameraStatus::SUCCESS)

    // Register callbacks.
    if (camera_->registerDataListener(depth_data_handler_.get()) !=
        royale::CameraStatus::SUCCESS) {
      throw std::logic_error("Error registering depth data listener");
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
  }
}

PicoFlexDriver::DepthImage PicoFlexDriver::GetDepthImage() const {
  if (!camera_) {
    throw std::logic_error("Camera is not catpuring.");
  }

  PicoFlexDriver::DepthImage ret;

  royale::DepthImage latest = depth_img_handler_->CopyLatestDepthImage(&ret.id);
  if (latest.width * latest.height == 0) {
    throw std::logic_error("Empty Frame.");
  }

  ret.timestamp = latest.timestamp;
  ret.depth_image = cv::Mat(latest.height, latest.width, CV_32F);
  ret.confidence_image = cv::Mat(latest.height, latest.width, CV_32F);
  for (uint16_t i = 0; i < latest.height; i++) {
    for (uint16_t j = 0; j < latest.width; j++) {
      uint16_t data = latest.data.at(i * latest.width + j);
      uint16_t z = data & 0x1fff;
      uint16_t confidence = (data >> 13) & 0x7;
      ret.depth_image.at<float>(i, j) = static_cast<float>(z) / 1e3;
      ret.confidence_image.at<float>(i, j) = static_cast<float>(confidence) / 1e3;
    }
  }

  return ret;
}
}
