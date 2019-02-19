#include <stdio.h>
#include <opencv2/opencv.hpp>

int mouse_X, mouse_Y;
std::vector<cv::Point> points;

void on_mouse(int evt, int x, int y, int flags, void* param) {
   if(evt == CV_EVENT_LBUTTONDOWN) {
       mouse_X = x;
       mouse_Y = y;
   }
}

cv::Rect calibrate_camera(cv::Mat frame)
{
  cv::namedWindow("ImageDisplay", 1);
  cv::setMouseCallback("ImageDisplay", on_mouse, (void*)&points);

  std::cout << "Please click on Top Left Corner. Then press space to Continue." << "\n";
  cv::imshow("ImageDisplay", frame);
  cv::waitKey(0);
  cv::Point tl(mouse_X, mouse_Y);
  std::cout << "Point: " << mouse_X << " " << mouse_Y << "\n";

  std::cout << "Please click on Bottom Right Corner. Then press space to Continue." << "\n";
  cv::imshow("ImageDisplay", frame);
  cv::waitKey(0);
  cv::Point br(mouse_X, mouse_Y);
  std::cout << "Point: " << mouse_X << " " << mouse_Y << "\n";

  cv::destroyWindow("ImageDisplay");
  return cv::Rect(tl,br);
}

std::vector<cv::Point2f> track_balls(cv::Rect calibrationRect)
{
  std::cout << calibrationRect.tl().x << " " << calibrationRect.tl().y << "\n";
  std::cout << calibrationRect.br().x << " " << calibrationRect.br().y << "\n";
  return std::vector<cv::Point2f>();
}

int main(int argc, char** argv)
{
  cv::VideoCapture video(0);
  cv::setMouseCallback("ImageDisplay", on_mouse, (void*)&points);

  cv::Mat frame;
  cv::Mat frame_gray;
  cv::Mat frame_prev;
  cv::namedWindow("Plinko", CV_WINDOW_AUTOSIZE);

  char function{'n'};

  cv::Mat output_image;

  cv::Rect calibrationRect(cv::Point(-1,-1),cv::Point(-1,-1));
  std::vector<cv::Point2f> ballLocations;

  for(;;)
  {

    video >> frame;
    cvtColor(frame, frame_gray, CV_BGR2GRAY);
    frame_prev = frame.clone();

    if(calibrationRect.tl().x!=-1)
      cv::rectangle(frame, calibrationRect, cv::Scalar(0, 255, 0));
    cv::imshow("Plinko", frame);
    char c = cv::waitKey(30);

    if(c=='c')
      calibrationRect = calibrate_camera(frame);
    else if(calibrationRect.tl().x!=-1)
      ballLocations = track_balls(calibrationRect);

    if(c=='q')
      break;

  }


  // cv::waitKey(0);

  return 0;
}
