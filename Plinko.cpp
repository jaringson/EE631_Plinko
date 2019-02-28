#include <stdio.h>
#include <opencv2/opencv.hpp>

int mouse_X, mouse_Y;
std::vector<cv::Point> points;
int numOfPegs = 11;

void on_mouse(int evt, int x, int y, int flags, void* param) {
   if(evt == cv::EVENT_LBUTTONDOWN) { //CV_EVENT_LBUTTONDOWN
       mouse_X = x;
       mouse_Y = y;
   }
}

void calibrate_camera(cv::Mat frame, cv::Rect& calibrationRect, std::vector<cv::Point2f>& pegs)
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

  calibrationRect = cv::Rect(tl,br);

  // std::vector<cv::Point> pegs;
  for(int i=0;i<numOfPegs;i++)
  {
    std::cout << "Please click on " << std::to_string(i+1) << " peg. Then press space to Continue." << "\n";
    cv::imshow("ImageDisplay", frame);
    cv::waitKey(0);
    pegs.push_back(cv::Point(mouse_X, mouse_Y));
    std::cout << "Point: " << mouse_X << " " << mouse_Y << "\n";
  }

  cv::destroyWindow("ImageDisplay");
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
  cv::namedWindow("Plinko", cv::WINDOW_AUTOSIZE); //CV_WINDOW_AUTOSIZE

  cv::Mat output_image;

  cv::Rect calibrationRect; 
  std::vector<cv::Point2f> ballLocations;
  std::vector<cv::Point2f> pegs;


  cv::FileStorage fs("../calibration.yaml",cv::FileStorage::READ);
  fs["CalibrationRectangle"] >> calibrationRect;
  fs["Pegs"] >> pegs;
  fs.release();

  for(;;)
  {

    video >> frame;
    cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY); //CV_BGR2GRAY
    frame_prev = frame.clone();

    cv::imshow("Plinko", frame);
    char c = cv::waitKey(30);

    if(c=='c')
      calibrate_camera(frame, calibrationRect, pegs);
    else if(calibrationRect.tl().x!=-1)
      ballLocations = track_balls(calibrationRect);
    if(c=='q')
      break;
    if(c=='s')
    {
      cv::FileStorage fs("../calibration.yaml",cv::FileStorage::WRITE);
      fs << "CalibrationRectangle" << calibrationRect;
      fs << "Pegs" << pegs;
      fs.release();
    }

    if(calibrationRect.tl().x!=-1)
    {
      cv::rectangle(frame, calibrationRect, cv::Scalar(0, 255, 0));
      for(int i=0;i<numOfPegs;i++)
      {
        cv::circle(frame, pegs[i], 2, cv::Scalar(0, 255, 0));
      }
      cv::imshow("Calibrated Plinko", frame);
      cv::waitKey(30);
    }

  }


  // cv::waitKey(0);

  return 0;
}
