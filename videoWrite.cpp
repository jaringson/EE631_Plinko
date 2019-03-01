#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

#include <vector>

using namespace cv;

int main()
{
  VideoCapture video(1);
  if(!video.isOpened())
      std::cout << "Cannot open camera\n";

  Mat frame;

  // int ex = static_cast<int>(video.get(CAP_PROP_FOURCC))
  int ex = VideoWriter::fourcc('M', 'J', 'P', 'G');
  Size size(video.get(3), video.get(4));
  VideoWriter v_out("plinko_up_board.avi", ex, 30, size, true); //1196444237 the big number is the FourCC code for MJPG got it from python script

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
