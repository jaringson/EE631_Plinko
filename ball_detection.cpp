#include <opencv2/opencv.hpp>

// Hue values of basic colors from https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
//
//         Orange  0-22
//         Yellow 22- 38
//         Green 38-75
//         Blue 75-130
//         Violet 130-160
//         Red 160-179


int main()
{
  cv::Mat img, hsv;
  img = cv::imread("plinko.jpg");

  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV); //Easier to detect color in HSV

  return 0;
}
