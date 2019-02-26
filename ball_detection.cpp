#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

// Hue values of basic colors from https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
//
// On test img: values may need to be adjusted due to glare
//  Red: H:0-120, S: 0-122, V: 189-255
//  Blue: H: 170-179, S: 150-205:, V: 120-160
// Green: H: 107-179, S: 200-255, V:0-130


int main()
{
  int low_h, high_h, low_s, high_s, low_v, high_v;

  cv::namedWindow("Control", cv::WINDOW_AUTOSIZE);

  cv::createTrackbar("LowH", "Control", &low_h, 179);
  cv::createTrackbar("HighH", "Control", &high_h, 179);
  cv::createTrackbar("LowS", "Control", &low_s, 255);
  cv::createTrackbar("HighS", "Control", &high_s, 255);
  cv::createTrackbar("LowV", "Control", &low_v, 255);
  cv::createTrackbar("HighV", "Control", &high_v, 255);

  // cv::VideoCapture cap(0);
  cv::Mat img, hsv;
  img = cv::imread("plinko.jpg");
  // img = cv::imread("test.jpg");

  cv::Mat img_red, img_blue, img_green;
  // int red_low(160), red_high(179);
  // int blue_low(75), blue_high(130);
  // int green_low(38), green_high(75);

  while(true)
  {
    // cap >> img;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV); //Easier to detect color in HSV
    // cv::inRange(img, cv::Scalar(red_low, low_s, low_v), cv::Scalar(red_high, high_s, high_v), img_red);
    cv::inRange(img, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v), img_red);

    cv::imshow("Original", img);
    cv::imshow("Red", img_red);
    if(cv::waitKey(30) == 27) //press esc to exit
      break;
}

  return 0;
}
