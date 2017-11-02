#include "pico_flex_driver.h"
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

void CloudVizLoop(const pico_flex_driver::PicoFlexDriver *driver) {
  uint64_t timestamp;

  pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");
  viewer.addCoordinateSystem(0.2, Eigen::Affine3f::Identity());

  while (true) {
    auto cloud = driver->GetPointCloud(&timestamp);
    if (cloud) {
      viewer.addPointCloud(cloud, "cloud");
      viewer.spinOnce();
      viewer.removePointCloud("cloud");
    }
  }
}

void DepthImgVizLoop(const pico_flex_driver::PicoFlexDriver *driver) {
  uint64_t timestamp;
  cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
  cv::Mat tmp;

  while (true) {
    auto img = driver->GetDepthImage(&timestamp);
    if (img) {
      // img is in mm, need to scale it to be better seen.
      img->convertTo(tmp, CV_8UC1, 255. / 1000., 0);

      cv::imshow("Depth Image", tmp);
      cv::waitKey(5);
    }
  }
}

int main() {
  pico_flex_driver::PicoFlexDriver driver;
  driver.StartCapture();

  std::thread img_thread(DepthImgVizLoop, &driver);
  std::thread cloud_thread(CloudVizLoop, &driver);

  img_thread.join();
  cloud_thread.join();

  return 0;
}
