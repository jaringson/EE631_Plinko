#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>

// Hue values of basic colors from https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
//
// On test img: values may need to be adjusted due to glare
// These values may work for RGB
//  Red: H:0-120, S: 0-122, V: 189-255
//  Blue: H: 170-179, S: 150-205:, V: 120-160
// Green: H: 107-179, S: 200-255, V:0-130

// These values are for HSV: Note that these may need to be adjusted for glare
//  Red: H:130-179, S:0-255, V: 0-255
//  Green: H: 60-100, S:50-255, V:0-255
//  Blue: H: 90-130, S:110-178, V: 0-255

cv::Mat cleanUpNoise(cv::Mat noisy_img)
{
  cv::Mat img;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); //Maybe make this 3, 3
  cv::erode(noisy_img, img, element);
  cv::dilate(img, img, element);

  return img;
}

std::vector<cv::Point2f> findCentroids(cv::Mat diff)
{
  //For some reason this finds the centroid twice
  cv::Mat canny_out;
  cv::Canny(diff, canny_out, 100, 200, 3); //May need to change the middle two values
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(canny_out, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Moments> mu(contours.size());
  std::vector<cv::Point2f> mc(contours.size());
  for(int i(0); i< contours.size(); i++)
  {
    mu[i] = cv::moments(contours[i]);
    mc[i] = cv::Point2f(static_cast<float>(mu[i].m10/(mu[i].m00+1e-5)),
                    static_cast<float>(mu[i].m01/(mu[i].m00+1e-5)));
  }

  return mc;
}

void printCentroids(std::vector<cv::Point2f> pts)
{
  for(cv::Point2f pt : pts)
    std::cout << pt << std::endl;
}

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

  cv::VideoCapture cap(1);
  cv::Mat img, hsv;
  // img = cv::imread("plinko.jpg");
  // img = cv::imread("test.jpg");

  cv::Mat img_red, img_blue, img_green;
  // int red_low(160), red_high(179);
  // int blue_low(75), blue_high(130);
  // int green_low(38), green_high(75);

  //For tuning
  while(true)
  {
    cap >> img;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV); //Easier to detect color in HSV
    // cv::inRange(img, cv::Scalar(red_low, low_s, low_v), cv::Scalar(red_high, high_s, high_v), img_red);
    cv::inRange(hsv, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v), img_red);

    img_red = cleanUpNoise(img_red);

    cv::imshow("Original", img);
    cv::imshow("Red", img_red);
    if(cv::waitKey(30) == 27) //press esc to exit
      break;
  }

  //For plinko.jpg
  // cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV); //Easier to detect color in HSV
  // cv::inRange(hsv, cv::Scalar(0, 130, 215), cv::Scalar(40, 255, 255), img_red);
  // cv::inRange(hsv, cv::Scalar(95, 70, 185), cv::Scalar(135, 255, 255), img_blue);
  // cv::inRange(hsv, cv::Scalar(50, 80, 170), cv::Scalar(90, 180, 255), img_green);
  //
  // // The following may not be necessary if our thresholding is good enough and we need to be faster
  // img_red = cleanUpNoise(img_red);
  // img_blue = cleanUpNoise(img_blue);
  // img_green = cleanUpNoise(img_green);
  //
  // std::vector<cv::Point2f> red_center = findCentroids(img_red);
  // std::vector<cv::Point2f> blue_center = findCentroids(img_blue);
  // std::vector<cv::Point2f> green_center = findCentroids(img_green);
  //
  // printCentroids(red_center);
  // printCentroids(blue_center);
  // printCentroids(green_center);
//
  // std::cout << red_center.size() << "\t" << blue_center.size() << "\t" << green_center.size() << "\n";
  //
  // cv::imshow("Red", img_red);
  // cv::imshow("Blue", img_blue);
  // cv::imshow("Green", img_green);
  // cv::imshow("Original", img);
  // cv::waitKey();

  return 0;
}
