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
cv::Mat cleanUpNoise(cv::Mat noisy_img);
std::vector<cv::Point2f> findCentroids(cv::Mat diff);

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
        Mat frame, hsv, img_red, img_blue, img_green;

        cap >> frame; // get a new frame from camera

        //Instead of drawing a rectangle, just crop the image
        // cv::rectangle(frame, calibrationRect, cv::Scalar(0, 255, 0));
        frame = frame(calibrationRect);

	      if(!frame.empty())
        {
            //ADD YOUR CODE HERE
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

            // These values are for HSV: Note that these may need to be adjusted for glare
            //  Red: H:130-179, S:0-255, V: 0-255
            //  Green: H: 60-100, S:50-255, V:0-255
            //  Blue: H: 90-130, S:110-178, V: 0-255
            //Threshold the image to isolate the ball
            cv::inRange(hsv, cv::Scalar(130, 0, 0), cv::Scalar(179, 255, 255), img_red);
            cv::inRange(hsv, cv::Scalar(60, 50, 0), cv::Scalar(100, 255, 255), img_green);
            cv::inRange(hsv, cv::Scalar(90, 110, 0), cv::Scalar(130, 178, 255), img_blue);

            //Clean up the noise
            img_red = cleanUpNoise(img_red);
            img_green = cleanUpNoise(img_green);
            img_blue = cleanUpNoise(img_blue);

            //Find centroids: Will this cause issues if there isn't a center?
            cv::Point2f red_center = findCentroids(img_red)[0];
            cv::Point2f blue_center = findCentroids(img_blue)[0];
            cv::Point2f green_center = findCentroids(img_green)[0];

            //Implement stragegy: will probably return a number indicating what column to go to.

        		imshow("Camera Input", frame);
        		if(waitKey(10) >= 0) break;

            // Command structure is very simple
            // "h\n" is to home the motor
            // "g<integer range 7 to 53>\n" sends the motor to that position in cm
            // e.g. "g35\n" sends the motor to 35cm from left wall
            //Good to know how to use
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
