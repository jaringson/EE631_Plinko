#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

#include <vector>

using namespace cv;

int main()
{
  VideoCapture video(0);
  if(!video.isOpened())
      std::cout << "Cannot open camera\n";

  Mat frame;

  // int ex = static_cast<int>(video.get(CAP_PROP_FOURCC))
  int ex = VideoWriter::fourcc('M', 'J', 'P', 'G');
  Size size(video.get(3), video.get(4));
  VideoWriter v_out("plinko_lights_board3.avi", ex, 30, size, true); //1196444237 the big number is the FourCC code for MJPG got it from python script

  if(!v_out.isOpened())
  {
    std::cout << "Failed to open video\n";
    return -1;
  }

  int key, mode(0);
  while(true)
  {
    //1280x720 resolution
    video >> frame;

    key = waitKey(30);
    if(key == (int)('q'))
      break;

    imshow("Forsgren", frame);

    v_out << frame;
  }
  v_out.release();
  video.release();

  return 0;
}


// int main(int argc, char** argv)
// {
//   cv::Mat img, hsv;
//   // img = cv::imread("color_values.jpg");
//   // // cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
//
//   cv::VideoCapture cap("plinko_lights_board1.avi");
//   cv::Mat frame, init_frame;
//   cap >> init_frame;
//   while(true)
//   {
//     cap >> frame;
//     if(frame.empty())
//       break;
//
//     cv::absdiff(init_frame, frame, frame);
//     cv::imshow("Frame", frame);
//     int key = cv::waitKey(30);
//     if(key == (int)('s'))
//       break;
//   }
//
//   hsv = frame;
//   // cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);
//
//   cv::namedWindow("ImageDisplay", 1);
//   cv::setMouseCallback("ImageDisplay", on_mouse, (void*)&points);
//
//   std::cout << "Please click on Red ball. Then press space to Continue." << "\n";
//   cv::imshow("ImageDisplay", hsv);
//   cv::waitKey(0);
//   cv::Mat mask;
//   cv::inRange(hsv, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), mask);
//   cv::bitwise_and(hsv, hsv, mask=mask);
//   cv::imshow("Blue", hsv);
//   cv::waitKey(0);
//   // cv::Point tl(mouse_Y, mouse_X); //switching x and y didnt work
//   cv::Vec3b color = hsv.at<cv::Vec3b>(mouse_X, mouse_Y);
//   // std::cout << "\nH_r\t" << (int)color.val[0] << "\tS_r\t" << (int)color.val[1] << "\tV_r\t" << (int)color.val[2];
//
//   std::cout << "\nPlease click on Blue ball. Then press space to Continue." << "\n";
//   cv::waitKey(0);
//   cv::Point tf(mouse_X, mouse_Y);
//   color = hsv.at<cv::Vec3b>(mouse_X, mouse_Y);
//   std::cout << "\nH_b\t" << (int)color.val[0] << "\tS_b\t" << (int)color.val[1] << "\tV_b\t" << (int)color.val[2];
//
//   std::cout << "\nPlease click on Green ball. Then press space to Continue." << "\n";
//   cv::waitKey(0);
//   cv::Point tg(mouse_X, mouse_Y);
//   color = hsv.at<cv::Vec3b>(mouse_X, mouse_Y);
//   std::cout << "\nH_g\t" << (int)color.val[0] << "\tS_g\t" << (int)color.val[1] << "\tV_g\t" << (int)color.val[2];
//
//   return 0;
// }
