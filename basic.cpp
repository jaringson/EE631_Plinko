#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <iostream>

#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


int fd, n, i;
char buf[128] = "temp text";
int mouse_X, mouse_Y;
std::vector<cv::Point> points;
using namespace cv;

void sendCommand(const char* command);
int setupSerial();
void calibrate_camera(cv::Mat frame, cv::Rect& calibrationRect);
void calibrate_pegs(cv::Mat frame, std::vector<cv::Point2f>& pegs);
void on_mouse(int evt, int x, int y, int flags, void* param);
void prepImg(cv::Mat &g_init, cv::Mat &g_frame, cv::Mat &diff);
cv::Mat cleanUpNoise(cv::Mat noisy_img);
cv::SimpleBlobDetector::Params setupParams();
void sendMotorToCol(int col);

int main(int, char**)
{
    int frameCounter = 0;
    Mat frameLast, g_init;
    // VideoCapture cap(0); // open the default camera
    VideoCapture cap("plinko_lights_board1.avi");
    // setupSerial();
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    cap >> frameLast;
    cv::cvtColor(frameLast, g_init, cv::COLOR_BGR2GRAY);
//    sendCommand("h\n"); // Home the motor and encoder

    //Setting up our calibration stuff here
    cv::Rect calibrationRect(cv::Point(-1,-1),cv::Point(-1,-1));
    std::vector<cv::Point2f> pegs;

   std::cout << "Enter 'c' to calibrate. Hit another key to read in file:\n";
   // char key = cv::waitKey(30); // For some reason it is not waiting here
   // std::cout << key << std::endl;
   // if(key == 'c')
      calibrate_camera(frameLast, calibrationRect);
//    else
//    {
//      ////// For Reading in the calibration file
//      cv::FileStorage fs_in("calibration.yaml",cv::FileStorage::READ);
//      fs_in["CalibrationRectangle"] >> calibrationRect;
//      fs_in["Pegs"] >> pegs;
//      fs_in.release();
//    }

    ////// For savingin off the calibration file
//    cv::FileStorage fs_out("calibration.yaml",cv::FileStorage::WRITE);
//    fs_out << "CalibrationRectangle" << calibrationRect;
//    fs_out << "Pegs" << pegs;
//    fs_out.release();

   cv::Rect roi = cv::Rect(calibrationRect.tl().x,
     calibrationRect.tl().y,
     calibrationRect.br().x-calibrationRect.tl().x,
     calibrationRect.br().y-calibrationRect.tl().y);

    // Crop the initial image
    frameLast = frameLast(roi);
     g_init = g_init(roi);
     calibrate_pegs(frameLast, pegs);

    //Set up blob detector
    cv::SimpleBlobDetector::Params params = setupParams();
    cv::Ptr<cv::SimpleBlobDetector> detect = cv::SimpleBlobDetector::create(params);

    for(;;)
    {
        frameCounter++;
        Mat frame, g_frame, diff;

        cap >> frame; // get a new frame from camera

	      if(!frame.empty())
        {
            //ADD YOUR CODE HERE
            cv::cvtColor(frame, g_frame, cv::COLOR_BGR2GRAY);
            frame = frame(roi);
            g_frame = g_frame(roi);

            prepImg(g_init, g_frame, diff);

            std::vector<cv::KeyPoint> keypts;
            detect->detect(diff, keypts);

            std::vector<cv::Point2f> centers;
            std::vector<cv::Point2f> balls(3); //red is first, blue is second, green is third
            cv::KeyPoint::convert(keypts, centers);

            for(cv::Point2f circle : centers)
            {
              cv::Vec3b color = frame.at<cv::Vec3b>(circle.y, circle.x);
              int b(color.val[0]), g(color.val[1]), r(color.val[2]);

              // if((b < 255 && b > 50) && (g < 255 && g > 50) && (r < 250 && r>200)) //would help get rid of some blobs
              if(r > b && r > g)
              {
                cv::circle(frame, circle, 5, cv::Scalar(255, 0, 0), -1);
                balls[0] = circle;
              }
              else if(b > r && b > g)
              {
                cv::circle(frame, circle, 5, cv::Scalar(0, 255, 0), -1);
                balls[2] = circle;
              }
              else if(g > r && g > b)
              {
                cv::circle(frame, circle, 5, cv::Scalar(0, 0, 255), -1);
                balls[1] = circle;
              }
            }

            //Implement stragegy: will probably return a number indicating what column/position to go to.
            //Can implement default to just go after the ball with the highest points
            //Column:cm : 1:6cm, 2:11cm, 3:16cm, 4:21cm, 5:26cm, 6:31cm, 7:36cm, 8:41cm, 9:46/47cm, 10:51/52cm

        		imshow("Camera Input", frame);
                char key;
        		key = waitKey(10);
                if (key == 'q')
                    sendMotorToCol(1);
                else if (key == 'w')
                    sendMotorToCol(2);
                else if (key == 'e')
                    sendMotorToCol(3);
                else if (key == 'r')
                    sendMotorToCol(4);
                else if (key == 't')
                    sendMotorToCol(5);
                else if (key == 'y')
                    sendMotorToCol(6);
                else if (key == 'u')
                    sendMotorToCol(7);
                else if (key == 'i')
                    sendMotorToCol(8);
                else if (key == 'o')
                    sendMotorToCol(9);
                else if (key == 'p')
                    sendMotorToCol(10);
		            else if (key == 'b')
		                break;

            // Command structure is very simple
            // "h\n" is to home the motor
            // "g<integer range 7 to 53>\n" sends the motor to that position in cm
            // e.g. "g35\n" sends the motor to 35cm from left wall
//            if(frameCounter%200==0)
//            {
//                sendCommand("g10\n");
//            }
//            else if(frameCounter%100==0)
//            {
//                sendCommand("g50\n");
//            }
        }
        else
          break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

void sendCommand(const char* command)
{
    printf("Sending Command: %s", command);
    /* Send byte to trigger Arduino to send string back */
    write(fd, command, strlen(command));
    //Receive string from Arduino
    n = read(fd, buf, 64);
    //insert terminating zero in the string
    buf[n] = 0;
}

int setupSerial()
{

    struct termios toptions;

    /* open serial port */
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    printf("fd opened as %i\n", fd);

    /* wait for the Arduino to reboot */
    usleep(1500000);


    /* get current serial port settings */
    tcgetattr(fd, &toptions);
    /* set 115200 baud both ways */
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);
    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    /* Canonical mode */
    toptions.c_lflag |= ICANON;
    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);
    printf("Attempting to communicate with arduino... \n");

    return 0;
}

void calibrate_camera(cv::Mat frame, cv::Rect& calibrationRect)
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

  cv::destroyWindow("ImageDisplay");
}

void calibrate_pegs(cv::Mat frame, std::vector<cv::Point2f>& pegs)
{
  cv::namedWindow("ImageDisplay", 1);
  cv::setMouseCallback("ImageDisplay", on_mouse, (void*)&points);

  cv::imshow("ImageDisplay", frame);
  cv::waitKey(30);

  int numOfPegs(11);  //Note that we need to do this after the iamge is cropped
  for(int i=0;i<numOfPegs;i++)
  {
    std::cout << "Please click on peg " << std::to_string(i+1) << ". Then press space to Continue." << "\n";
    cv::waitKey(0);
    pegs.push_back(cv::Point2f(mouse_X, mouse_Y));
    std::cout << "Point: " << mouse_X << " " << mouse_Y << "\n";
  }

  cv::destroyWindow("ImageDisplay");
}

void prepImg(cv::Mat &g_init, cv::Mat &g_frame, cv::Mat &diff)
{
  cv::absdiff(g_init, g_frame, diff);
  cv::threshold(diff, diff, 40, 255, 0);
  diff = cleanUpNoise(diff);
}

void on_mouse(int evt, int x, int y, int flags, void* param)
{
   if(evt == cv::EVENT_LBUTTONDOWN) { //CV_EVENT_LBUTTONDOWN
       mouse_X = x;
       mouse_Y = y;
   }
}

cv::Mat cleanUpNoise(cv::Mat noisy_img)
{
  cv::Mat img;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); //Maybe make this 3, 3
  cv::erode(noisy_img, img, element);
  cv::dilate(img, img, element);

  return img;
}

cv::SimpleBlobDetector::Params setupParams()
{
  cv::SimpleBlobDetector::Params params;
  params.minThreshold = 100;
  params.maxThreshold = 255; //maybe try by circularity also
  params.filterByColor = true;
  params.blobColor = 255;
  params.filterByArea = true;
  params.minArea = 153;
  params.maxArea = 1256;

  return params;
}

void sendMotorToCol(int col)
{
    if (col == 5)
        sendCommand("g25\n");
    else if (col == 6)
        sendCommand("g30\n");
    else if (col == 4)
        sendCommand("g20\n");
    else if (col == 7)
        sendCommand("g36\n");
    else if (col == 3)
        sendCommand("g15\n");
    else if (col == 8)
        sendCommand("g41\n");
    else if (col == 2)
        sendCommand("g10\n");
    else if (col == 9)
        sendCommand("g46\n");
    else if (col == 1)
        sendCommand("h\n");
    else if (col == 10)
        sendCommand("g51\n");
    else
        sendCommand("g24\n");
}
