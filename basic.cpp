#include <opencv2/opencv.hpp>
#include <vector>



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
cv::Rect calibrate_camera(cv::Mat frame);
void on_mouse(int evt, int x, int y, int flags, void* param);

int main(int, char**)
{   int frameCounter = 0;
    Mat frameLast;
    VideoCapture cap(0); // open the default camera
    setupSerial();
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    cap >> frameLast;
    sendCommand("h\n"); // Home the motor and encoder

    //Setting up our calibration stuff here
    cv::Rect calibrationRect(cv::Point(-1,-1),cv::Point(-1,-1));
    calibrationRect = calibrate_camera(frameLast);

    for(;;)
    {
        frameCounter++;
        Mat frame;

        cap >> frame; // get a new frame from camera
        cv::rectangle(frame, calibrationRect, cv::Scalar(0, 255, 0));
        
	      if(!frame.empty()){
        //ADD YOUR CODE HERE


    		imshow("Camera Input", frame);
    		if(waitKey(10) >= 0) break;

        // Command structure is very simple
        // "h\n" is to home the motor
        // "g<integer range 7 to 53>\n" sends the motor to that position in cm
        // e.g. "g35\n" sends the motor to 35cm from left wall
        if(frameCounter%200==0)
        {
            sendCommand("g10\n");
        }
        else if(frameCounter%100==0)
        {
            sendCommand("g50\n");
        }
	}
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

void on_mouse(int evt, int x, int y, int flags, void* param)
{
   if(evt == cv::EVENT_LBUTTONDOWN) { //CV_EVENT_LBUTTONDOWN
       mouse_X = x;
       mouse_Y = y;
   }
}
