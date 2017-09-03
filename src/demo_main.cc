#include "driver.h"
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>

int main() {
  pico_flex_driver::PicoFlexDriver driver;

  driver.StartCapture();

  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  usleep(1e6);

  pico_flex_driver::PicoFlexDriver::DepthImage img;
  uint64_t last_ctr = 0;

  while (true) {
    img = driver.GetDepthImage();
    if (img.id == last_ctr) {
      usleep(1e4);
      continue;
    }

    cv::imshow("Display window", img.image);
    cv::waitKey(1);
  }

  return 0;
}
