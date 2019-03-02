#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <iostream>

#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

using namespace cv;

int fd, n, i;
char buf[128] = "temp text";
int mouse_X, mouse_Y;
std::vector<cv::Point> points;

void sendCommand(const char* command);
int setupSerial();
void calibrate_camera(cv::Mat frame, cv::Rect& calibrationRect);
void calibrate_pegs(cv::Mat frame, std::vector<cv::Point2f>& pegs);
void on_mouse(int evt, int x, int y, int flags, void* param);
void prepImg(cv::Mat &g_init, cv::Mat &g_frame, cv::Mat &diff);
cv::Mat cleanUpNoise(cv::Mat noisy_img);
cv::SimpleBlobDetector::Params setupParams();
void drawCircles(cv::Mat& frame, std::vector<cv::Point2f> &centers,
                 std::vector<cv::Point2f> balls);
void sendMotorToCol(int col);
int overrideMotorWithKeyPress(char key, int& col_cmd);

// catching options
int getColFromPixel(int x_pixel,const std::vector<cv::Point2f>& pegs);
int catchRedBall(const std::vector<cv::Point2f>& balls, const
                  std::vector<cv::Point2f>& pegs);
int getLowestBall(const std::vector<cv::Point2f>& centers,
                  const std::vector<cv::Point2f>& pegs);

int main(int, char**)
{
    int frameCounter = 0;
    int col_cmd{5};
    int prev_col_cmd{0};
    Mat frameLast, g_init;
//    VideoCapture cap(0); // open the default camera
    VideoCapture cap("plinko_lights_board3.avi");
//    setupSerial();
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    cap >> frameLast;
    cv::cvtColor(frameLast, g_init, cv::COLOR_BGR2GRAY);
    // sendCommand("h\n"); // Home the motor and encoder

    //Setting up our calibration stuff here
    cv::Rect calibrationRect(cv::Point(-1,-1),cv::Point(-1,-1));
    std::vector<cv::Point2f> pegs;
    cv::Rect roi;

    std::cout << "Enter 'c' to calibrate. Hit another key to read in file:" << std::endl;
    cv::imshow("Temp", frameLast);
    int key = cv::waitKey(0); // For some reason it is not waiting here
    std::cout << key << std::endl;
    if(key == (int)('c'))
        calibrate_camera(frameLast, calibrationRect);
    else
    {
        //// For Reading in the calibration file
        cv::FileStorage fs_in("calibration.yaml",cv::FileStorage::READ);
        fs_in["CalibrationRectangle"] >> calibrationRect;
        fs_in["Pegs"] >> pegs;
        fs_in.release();
    }

    roi = cv::Rect(calibrationRect.tl().x,
    calibrationRect.tl().y,
    calibrationRect.br().x-calibrationRect.tl().x,
    calibrationRect.br().y-calibrationRect.tl().y);

    // Crop the initial image
    frameLast = frameLast(roi);
    g_init = g_init(roi);
    if(key == (int)('c'))
        calibrate_pegs(frameLast, pegs);

    //// For saving off the calibration file
    cv::FileStorage fs_out("calibration.yaml",cv::FileStorage::WRITE);
    fs_out << "CalibrationRectangle" << calibrationRect;
    fs_out << "Pegs" << pegs;
    fs_out.release();

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

            std::vector<cv::Point2f> balls;
            cv::KeyPoint::convert(keypts, balls);

            //Draw circle on the balls
            //If there is no center detected the Point will show 0, 0
            std::vector<cv::Point2f> centers(3); //red is first, blue is second, green is third
            drawCircles(frame, centers, balls);

            // go for red ball only (highest point ball)
            // if (balls.size() == 3)
            // {
            //     col_cmd = catchRedBall(centers, pegs);
            //     std::cout << col_cmd << std::endl;
            // }
            col_cmd = getLowestBall(centers, pegs);
            std::cout << col_cmd << std::endl;

            imshow("Camera Input", frame);
            char key;
            key = waitKey(1);

            overrideMotorWithKeyPress(key, col_cmd);
            if (key == 'q')
	            break;

            // if (col_cmd != prev_col_cmd)
                // sendMotorToCol(col_cmd);
            prev_col_cmd = col_cmd;
        }
        else
            break;
    }
    return 0; // end of main
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

    int numOfPegs(11); //Note that we need to do this after the iamge is cropped
    for(int i=0;i<numOfPegs;i++)
    {
        std::cout << "Please click on peg " << std::to_string(i+1) <<
                     ". Then press space to Continue." << "\n";
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
    if(evt == cv::EVENT_LBUTTONDOWN)
    { //CV_EVENT_LBUTTONDOWN
        mouse_X = x;
        mouse_Y = y;
    }
}

cv::Mat cleanUpNoise(cv::Mat noisy_img)
{
    cv::Mat img;

    //Maybe make this 3, 3
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
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

void drawCircles(cv::Mat& frame, std::vector<cv::Point2f> &centers, std::vector<cv::Point2f> balls)
{
    for(cv::Point2f circle : balls)
    {
        cv::Vec3b color = frame.at<cv::Vec3b>(circle.y, circle.x);
        int b(color.val[0]), g(color.val[1]), r(color.val[2]);

    // if((b < 255 && b > 50) && (g < 255 && g > 50) && (r < 250 && r>200)) //would help get rid of some blobs
        if(r > b && r > g)
        {
            cv::circle(frame, circle, 20, cv::Scalar(0, 0, 255), 1, 8);
            centers[0] = circle;
        }
        else if(b > r && b > g)
        {
            cv::circle(frame, circle, 20, cv::Scalar(255, 0, 0), 1, 8);
            centers[2] = circle;
        }
        else if(g > r && g > b)
        {
            cv::circle(frame, circle, 20, cv::Scalar(0, 255, 0), 1, 8);
            centers[1] = circle;
        }
    }
}

void sendMotorToCol(int col)
{
    // Command structure is very simple
    // "h\n" is to home the motor
    // "g<integer range 7 to 53>\n" sends the motor to that position in cm
    // e.g. "g35\n" sends the motor to 35cm from left wall
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
        sendCommand("g25\n");
}

int overrideMotorWithKeyPress(char key, int& col_cmd)
{
    if (key == '1')
        col_cmd = 1;
    else if (key == '2')
        col_cmd = 2;
    else if (key == '3')
        col_cmd = 3;
    else if (key == '4')
        col_cmd = 4;
    else if (key == '5')
        col_cmd = 5;
    else if (key == '6')
        col_cmd = 6;
    else if (key == '7')
        col_cmd = 7;
    else if (key == '8')
        col_cmd = 8;
    else if (key == '9')
        col_cmd = 9;
    else if (key == '0')
        col_cmd = 10;
}

// TODO check if it is really x that is needed
int getColFromPixel(int x_pixel,const std::vector<cv::Point2f>& pegs)
{
    int col;
    if (x_pixel < pegs[5].x)
    {
        if (x_pixel > pegs[4].x)
            col = 5;
        else if (x_pixel > pegs[3].x)
            col = 4;
        else if (x_pixel > pegs[2].x)
            col = 3;
        else if (x_pixel > pegs[1].x)
            col = 2;
        else
            col = 1;
    }
    else
    {
        if (x_pixel < pegs[6].x)
            col = 6;
        else if (x_pixel < pegs[7].x)
            col = 7;
        else if (x_pixel < pegs[8].x)
            col = 8;
        else if (x_pixel < pegs[9].x)
            col = 9;
        else
            col = 10;
    }

    return col;
}

int catchRedBall(const std::vector<cv::Point2f>& balls, const
                  std::vector<cv::Point2f>& pegs)
{
   int col_cmd{getColFromPixel(balls[0].x, pegs)};

   return col_cmd;
}

int getLowestBall(const std::vector<cv::Point2f>& centers, const std::vector<cv::Point2f>& pegs)
{
  double r_y{centers[0].y}, b_y{centers[2].y}, g_y{centers[1].y};

  //Check if the center was actually found
  if(r_y == 0.0)
    r_y = 1000.0;
  if(b_y == 0.0)
    b_y = 1000.0;
  if(g_y == 0.0)
    g_y = 1000.0;

    int index;
    if(r_y < b_y && r_y < g_y)
      index = 0;
    else if(g_y < r_y && g_y < b_y)
      index = 1;
    else //(b_y < r_y && b_y < g_y)
      index = 2;

    int col_cmd{getColFromPixel(centers[index].x, pegs)};

    return col_cmd;
}
