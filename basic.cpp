#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <iostream>
#include <cmath>

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
void prepImg(cv::Mat &g_init, cv::Mat &g_frame, cv::Mat frame_init, cv::Mat frame,
  cv::Mat &red_diff, cv::Mat &green_diff, cv::Mat &blue_diff);
cv::Mat cleanUpNoise(cv::Mat noisy_img);
cv::SimpleBlobDetector::Params setupParams();
void drawCircles(cv::Mat& frame, std::vector<cv::Point2f> &centers,
   std::vector<cv::Point2f> red_balls,
   std::vector<cv::Point2f> blue_balls,
   std::vector<cv::Point2f> green_balls, int max_y);
void sendMotorToCol(int col);
int overrideMotorWithKeyPress(char key, int& col_cmd);

// catching options
int getColFromPixel(int x_pixel,const std::vector<cv::Point2f>& pegs);
void drawTuningLines(int col_cmd, std::vector<Point2f>& pegs, Mat& frame);
int catchRedBall(const std::vector<cv::Point2f>& centers, const
                  std::vector<cv::Point2f>& pegs);
int getLowestBall(const std::vector<cv::Point2f>& centers,
                  const std::vector<cv::Point2f>& pegs);
int getLowestPossibleBall(const std::vector<cv::Point2f>& centers, 
        const std::vector<cv::Point2f>& pegs, const int cur_col);

int main(int, char**)
{
    int frameCounter = 0;
    int col_cmd{5};
    int prev_col_cmd{0};
    Mat frameLast, g_init, frame_init;
//    VideoCapture cap(0); // open the default camera
    VideoCapture cap("plinko_w_motor5.avi");
//    setupSerial();
    if(!cap.isOpened())
    {  // check if we succeeded
        std::cout << "Failed to open video\n";
        return -1;
    }

//    int ex = VideoWriter::fourcc('M', 'J', 'P', 'G');
//    Size size(cap.get(3), cap.get(4));
//    VideoWriter v_out("plinko_w_motor5.avi", ex, 30, size, true);

    cap >> frameLast;
    cv::Mat diffs = cv::Mat::zeros(frameLast.size(), frameLast.type());
    diffs.convertTo(diffs, CV_32F);
    int key;
    int count{0};
    while(true)
    {
      cap >> frameLast;
      //v_out << frameLast;
      cv::imshow("Temp", frameLast);
      frameLast.convertTo(frameLast, CV_32F);
      std::cout << "Hit 's' if you want to save with as one of your background images\n";
      std::cout << "Hit 'q' if you want to quit saving background images\n";
      key = cv::waitKey(0);
      if(key == (int)('s'))
      {
        diffs = frameLast + diffs;
        count++;
      }
      if(key == (int)('q'))
          break;
    }
    frameLast = diffs/count;
    frameLast.convertTo(frameLast, CV_8U);
    // diffs.convertTo(diffs, CV_8U);
    // cv::imshow("Last Frame", frameLast);
    // cv::imshow("Diffs", diffs);
    // key = cv::waitKey(0);
    cv::cvtColor(frameLast, g_init, cv::COLOR_BGR2GRAY);
//    sendCommand("h\n"); // Home the motor and encoder

    //Setting up our calibration stuff here
    cv::Rect calibrationRect(cv::Point(-1,-1),cv::Point(-1,-1));
    std::vector<cv::Point2f> pegs;
    cv::Rect roi;

    std::cout << "Enter 'c' to calibrate. Hit another key to read in file:" << std::endl;
    key = cv::waitKey(0); // For some reason it is not waiting here
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
    // frame_init = frameLast.clone();
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
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    for(;;)
    {
        frameCounter++;
        Mat frame, g_frame, red_diff, green_diff, blue_diff;

        cap >> frame; // get a new frame from camera
//        v_out << frame; // write frame to video for testing

        if(!frame.empty())
        {
            //ADD YOUR CODE HERE
            cv::cvtColor(frame, g_frame, cv::COLOR_BGR2GRAY);
            frame = frame(roi);
            g_frame = g_frame(roi);

            prepImg(g_init, g_frame, frameLast.clone(), frame.clone(),
                    red_diff, green_diff, blue_diff);

            std::vector<cv::KeyPoint> red_keypts;
            std::vector<cv::KeyPoint> blue_keypts;
            std::vector<cv::KeyPoint> green_keypts;
            detector->detect(red_diff, red_keypts);
            detector->detect(blue_diff, blue_keypts);
            detector->detect(green_diff, green_keypts);

            std::vector<cv::Point2f> red_balls;
            cv::KeyPoint::convert(red_keypts, red_balls);
            std::vector<cv::Point2f> blue_balls;
            cv::KeyPoint::convert(blue_keypts, blue_balls);
            std::vector<cv::Point2f> green_balls;
            cv::KeyPoint::convert(green_keypts, green_balls);

            //Draw circle on the balls
            //If there is no center detected the Point will show 0, 0
            std::vector<cv::Point2f> centers(3); //red is first, blue is second, green is third
            drawCircles(frame, centers, red_balls, blue_balls, green_balls,
                        pegs[0].y);

            // col_cmd = catchRedBall(centers, pegs);
            // col_cmd = getLowestBall(centers, pegs);
            col_cmd = getLowestPossibleBall(centers,pegs,col_cmd);
            //if (col_cmd != -1)
            //    std::cout << col_cmd << std::endl;

            col_cmd = (col_cmd == -1) ? prev_col_cmd : col_cmd;

            drawTuningLines(col_cmd, pegs, frame);

            imshow("Camera Input", frame);

            imshow("Red AbsDiff", red_diff);
            imshow("Green AbsDiff", green_diff);
            imshow("Blue AbsDiff", blue_diff);
            char key;
 //           key = waitKey(1);
            key = waitKey(0);

            overrideMotorWithKeyPress(key, col_cmd);
            if (key == 'q')
	            break;

//            if (col_cmd != prev_col_cmd)
//                sendMotorToCol(col_cmd);
            prev_col_cmd = col_cmd;
        }
        else
            break;
    }
//    v_out.release();
    destroyAllWindows();
    return 0; // end of main
}

void sendCommand(const char* command)
{
//    printf("Sending Command: %s", command);
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
    std::cout << "Hey\n";

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

void prepImg(cv::Mat &g_init, cv::Mat &g_frame, cv::Mat frame_init, cv::Mat frame,
  cv::Mat &red_diff, cv::Mat &green_diff, cv::Mat &blue_diff)
{
    cv::Mat diff;
    // std::cout << "here" << std::endl;
    // cv::imshow("bitwise frame", frame);
    // cv::imshow("bitwise frame init", frame_init);
    // cv::waitKey(0);

    cv::absdiff(frame_init, frame, diff);
    // cv::imshow("color diff", diff);

    cv::bitwise_and(frame, diff, frame);
    cv::inRange(frame, cv::Scalar(0,0,50), cv::Scalar(80,80,255), red_diff);
    cv::inRange(frame, cv::Scalar(0,80,0), cv::Scalar(80,255,80), green_diff);
    cv::inRange(frame, cv::Scalar(80,0,0), cv::Scalar(255,80,80), blue_diff);

    red_diff = cleanUpNoise(red_diff);
    green_diff = cleanUpNoise(green_diff);
    blue_diff = cleanUpNoise(blue_diff);
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
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6));
    cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    cv::erode(noisy_img, img, element);
    cv::dilate(noisy_img, img, dilate_element);

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
    params.minArea = 250;
    params.maxArea = 1256;
    params.filterByCircularity = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;

    return params;
}

void drawCircles(cv::Mat& frame, std::vector<cv::Point2f> &centers,
   std::vector<cv::Point2f> red_balls,
   std::vector<cv::Point2f> blue_balls,
   std::vector<cv::Point2f> green_balls, int max_y)
{
    static double alpha{0.6};

    //RED CIRCLE
    // int max_red{0};
    // cv::Point2f red_max_winner;

    static cv::Point2f red_winner_prev;
    int close_red{10000};
    cv::Point2f red_close_winner;

    for(cv::Point2f circle : red_balls)
    {
        cv::Vec3b color = frame.at<cv::Vec3b>(circle.y, circle.x);
        int b(color.val[0]), g(color.val[1]), r(color.val[2]);
        // if(r>max_red)
        // {
        //     max_red = r;
        //     red_max_winner = circle;
        // }
        if(abs(circle.x-red_winner_prev.x) < close_red)
        {
            close_red = abs(circle.x-red_winner_prev.x);
            red_close_winner = circle;
        }
        else if (red_winner_prev.x > max_y)
            red_close_winner = circle;
    }
    if(red_close_winner.x==0 && red_winner_prev.y < max_y)
    {
        cv::circle(frame, red_winner_prev, 20, cv::Scalar(0, 0, 255), 1, 8);
        centers[0] = red_winner_prev;
    }
    else if(red_close_winner.x==0 && red_winner_prev.y > max_y)
    {
        cv::circle(frame, red_close_winner, 20, cv::Scalar(0, 0, 255), 1, 8);
        centers[0] = red_close_winner;
    }
    else
    {
        cv::circle(frame, red_close_winner, 20, cv::Scalar(0, 0, 255), 1, 8);
        centers[0] = red_close_winner;
    }
    

    // std::cout << "RED Winner " << red_close_winner.x << std::endl;
    // std::cout << "RED Winner Prev " << red_winner_prev.x << std::endl;
    if(red_close_winner.x > 5)
      red_winner_prev = (1-alpha)*red_close_winner + alpha*red_winner_prev;

    //BLUE CIRCLE
    // int max_blue{0};
    // cv::Point2f blue_max_winner;

    static cv::Point2f blue_winner_prev;
    int close_blue{10000};
    cv::Point2f blue_close_winner;

    for(cv::Point2f circle : blue_balls)
    {
        cv::Vec3b color = frame.at<cv::Vec3b>(circle.y, circle.x);
        int b(color.val[0]), g(color.val[1]), r(color.val[2]);
        // if(b>max_blue)
        // {
        //     max_blue = b;
        //     blue_max_winner = circle;
        // }
        if(abs(circle.x-blue_winner_prev.x) < close_blue)
        {
            close_blue = abs(circle.x-blue_winner_prev.x);
            blue_close_winner = circle;
        }
        else if (blue_winner_prev.y > max_y)
            blue_close_winner = circle;
    }
    if(blue_close_winner.x ==0 && blue_winner_prev.y < max_y)
    {
        cv::circle(frame, blue_winner_prev, 20, cv::Scalar(255, 0, 0), 1, 8);
        centers[2] = blue_winner_prev;
    }
    else if(blue_close_winner.x ==0 && blue_winner_prev.y > max_y)
    {
        cv::circle(frame, blue_close_winner, 20, cv::Scalar(255, 0, 0), 1, 8);
        centers[2] = blue_close_winner;
    }
    else
    {
        cv::circle(frame, blue_close_winner, 20, cv::Scalar(255, 0, 0), 1, 8);
        centers[2] = blue_close_winner;
    }

    // std::cout << "BLUE Winner " << blue_close_winner.x << std::endl;
    // std::cout << "BLUE Winner Prev " << blue_winner_prev.x << std::endl;
    if(blue_close_winner.x > 5)
      blue_winner_prev = (1-alpha)*blue_close_winner + alpha*blue_winner_prev;

    //GREEN CIRCLE
    // int max_green{0};
    // cv::Point2f green_winner;

    static cv::Point2f green_winner_prev;
    int close_green{10000};
    cv::Point2f green_close_winner;

    for(cv::Point2f circle : green_balls)
    {
//        std::cout << "green circle found" << std::endl;
        cv::Vec3b color = frame.at<cv::Vec3b>(circle.y, circle.x);
        int b(color.val[0]), g(color.val[1]), r(color.val[2]);
        // if(g>max_green)
        // {
        //     max_green = g;
        //     green_winner = circle;
        // }
        if(abs(circle.x-green_winner_prev.x) < close_green)
        {
            close_green = abs(circle.x-green_winner_prev.x);
            green_close_winner = circle;
        }
        else if (green_winner_prev.y > max_y)
            green_close_winner = circle;
    }
    if(green_close_winner.x==0 && green_winner_prev.y < max_y)
    {
        cv::circle(frame, green_winner_prev, 20, cv::Scalar(0, 255, 0), 1, 8);
        centers[1] = green_winner_prev;
    }
    else if(green_close_winner.x==0 && green_winner_prev.y > max_y)
    {
        Point2f zero;
        cv::circle(frame, zero, 20, cv::Scalar(0, 255, 0), 1, 8);
        centers[1] = zero;
    }
    else
    {
        cv::circle(frame, green_close_winner, 20, cv::Scalar(0, 255, 0), 1, 8);
        centers[1] = green_close_winner;
    }


    // std::cout << "GREEN Winner " << green_close_winner.x << std::endl;
    // std::cout << "GREEN Winner Prev " << green_winner_prev.x << std::endl;
    if(green_close_winner.x > 5)
      green_winner_prev = (1-alpha)*green_close_winner + alpha*green_winner_prev;

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
        sendCommand("g7\n");
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

//TODO need to send in pegs and use pegs[col-k].x
void drawTuningLines(int col, std::vector<Point2f>& pegs, Mat& frame)
{
    int y{pegs[0].y + 35};
    line(frame, Point(pegs[col-1].x,y), Point(pegs[col].x,y),(255,255,255),2);

    int slope{5};
    int count{1};
    for (int k{col-1}; k > 0; k--)
    {
        int h{y - slope*count};
        line(frame,Point(pegs[k-1].x,h),Point(pegs[k].x,h),(255,255,255),2);
        count++;
    }
    count = 1;
    for (int k{col}; k < 10; k++)
    {
        int h{y - slope*count};
        line(frame,Point(pegs[k].x,h),Point(pegs[k+1].x,h),(255,255,255),2);
        count++;
    }
}

int catchRedBall(const std::vector<cv::Point2f>& centers, const
                  std::vector<cv::Point2f>& pegs)
{
    int col_cmd;
    if (centers[0].x)
        col_cmd = getColFromPixel(centers[0].x, pegs);
    else
        col_cmd = -1;

    return col_cmd;
}

int getLowestBall(const std::vector<cv::Point2f>& centers, const std::vector<cv::Point2f>& pegs)
{
    double lowest, next_lowest;
    double r_y{centers[0].y}, b_y{centers[2].y}, g_y{centers[1].y};

    //Check if the center was actually found
    if(r_y == 0.0 && b_y ==0.0 && g_y == 0.0)
        return -1;

    int index, next_index;
    if (r_y > b_y && r_y > g_y)
    {
        index = 0;
        lowest = r_y;
        if (g_y > b_y)
        {
            next_lowest = g_y;
            next_index = 1;
        }
        else
        {
            next_lowest = b_y;
            next_index = 2;
        }
    }
    else if(g_y > r_y && g_y > b_y)
    {
        index = 1;
        lowest = g_y;
        if (r_y > b_y)
        {
            next_lowest = r_y;
            next_index = 0;
        }
        else
        {
            next_lowest = b_y;
            next_index = 2;
        }
    }
    else //(b_y > r_y && b_y > g_y)
    {
        index = 2;
        lowest = b_y;
        if (r_y > g_y)
        {
            next_lowest = r_y;
            next_index = 0;
        }
        else
        {
            next_lowest = g_y;
            next_index = 1;
        }
    }

    if(lowest > pegs[0].y + 35)
    {
        index = next_index;
    }

    int col_cmd{getColFromPixel(centers[index].x, pegs)};
    if (index == 0)
        std::cout << "Lowest ball is red at " << r_y << std::endl;
    if (index == 1)
        std::cout << "Lowest ball is green at " << g_y << std::endl;
    if (index == 2)
        std::cout << "Lowest ball is blue at " << b_y << std::endl;

    if (next_index == 0)
        std::cout << "Next ball is red at " << r_y << std::endl;
    if (next_index == 1)
        std::cout << "Next ball is green at " << g_y << std::endl;
    if (next_index == 2)
        std::cout << "Next ball is blue at " << b_y << std::endl;

    std::cout << "Sending motor to column " << col_cmd << std::endl << std::endl;

    return col_cmd;
}

double maxDiff(double r_h, double g_h, double b_h)
{
    double rg_diff{r_h - g_h};
    rg_diff *= (rg_diff < 0) ? -1 : 1;
    double bg_diff{b_h - g_h};
    bg_diff *= (bg_diff < 0) ? -1 : 1;
    double rb_diff{r_h - b_h};
    rb_diff *= (rb_diff < 0) ? -1 : 1;

    double max_diff;
    if (rg_diff >= bg_diff && rg_diff >= rb_diff)
        max_diff = rg_diff;
    else if (bg_diff >= rg_diff && bg_diff >= rb_diff)
        max_diff = bg_diff;
    else // (rg_diff > bg_diff && rg_diff > rb_diff)
        max_diff = rb_diff;

    return max_diff;
}

int getLowestPossibleBall(const std::vector<cv::Point2f>& centers, 
        const std::vector<cv::Point2f>& pegs, const int cur_col)
{
    double lowest, next_lowest, highest;
    double r_y{centers[0].y}, b_y{centers[2].y}, g_y{centers[1].y};

    //Check if the center was actually found
    if(r_y == 0.0 && b_y ==0.0 && g_y == 0.0)
        return -1;

    if (maxDiff(r_y,g_y,b_y) < 60)
    {
        int col_cmd{getColFromPixel(centers[0].x, pegs)};
        std::cout << "Heights too similar, going for red ball." << std::endl;
        std::cout << "Sending motor to column " << col_cmd << std::endl << std::endl;
        return col_cmd;
    }

    int index, next_index, high_index;
    if(r_y > b_y && r_y > g_y)
    {
        index = 0;
        lowest = r_y;
        if (g_y > b_y)
        {
            next_lowest = g_y;
            next_index = 1;
            highest = b_y;
            high_index = 2;
        }
        else
        {
            next_lowest = b_y;
            next_index = 2;
            highest = g_y;
            high_index = 1;
        }
    }
    else if(g_y > r_y && g_y > b_y)
    {
        index = 1;
        lowest = g_y;
        if (r_y > b_y)
        {
            next_lowest = r_y;
            next_index = 0;
            highest = b_y;
            high_index = 2;
        }
        else
        {
            next_lowest = b_y;
            next_index = 2;
            highest = r_y;
            high_index = 0;
        }
    }
    else //(b_y > r_y && b_y > g_y)
    {
        index = 2;
        lowest = b_y;
        if (r_y > g_y)
        {
            next_lowest = r_y;
            next_index = 0;
            highest = g_y;
            high_index = 1;
        }
        else
        {
            next_lowest = g_y;
            next_index = 1;
            highest = r_y;
            high_index = 0;
        }
    }

    if (index == 0)
        std::cout << "Lowest ball is red at " << r_y << std::endl;
    if (index == 1)
        std::cout << "Lowest ball is green at " << g_y << std::endl;
    if (index == 2)
        std::cout << "Lowest ball is blue at " << b_y << std::endl;

    if (next_index == 0)
        std::cout << "Next ball is red at " << r_y << std::endl;
    if (next_index == 1)
        std::cout << "Next ball is green at " << g_y << std::endl;
    if (next_index == 2)
        std::cout << "Next ball is blue at " << b_y << std::endl;

    if (high_index == 0)
        std::cout << "Highest ball is red at " << r_y << std::endl;
    if (high_index == 1)
        std::cout << "Highest ball is green at " << g_y << std::endl;
    if (high_index == 2)
        std::cout << "Highest ball is blue at " << b_y << std::endl;

    // if a ball has crossed the line
    int max_y{pegs[0].y + 35}; // TODO tune
    if(lowest > max_y)
    {
        std::cout << "This ball fell below the line: " << index << std::endl;
        // if there are no more balls, keep same position
        if (next_lowest == 0 && highest == 0)
            return -1;
        // if there is only one ball left, try to get it
        else if (highest == 0)
        {
            std::cout << "Going for last ball" << std::endl;
            index = next_index;
        }
        // if there are 2 balls left check if it is possible to get all 3
        // of if the second ball is not catchable and needs to be skipped
        else
        {
            // TODO add check to see if height of remaining 2 are similar
            // then go for red only if they are
            
            int cur_col{getColFromPixel(centers[index].x, pegs)};
            int next_col{getColFromPixel(centers[next_index].x, pegs)};
            int col_diff{cur_col - next_col};
            col_diff *= (col_diff < 0) ? -1 : 1;

            // if 2nd ball is in the same column as the one that just fell 
            // then don't change columns
            if (col_diff == 0)
            {
                std::cout << "Sending motor to column " << cur_col << 
                              std::endl << std::endl;
                return cur_col;
            }

            // these function as a descrete line with the given slope
            int slope{5}; // TODO tune
            if (next_lowest < max_y-slope*1 && col_diff <=1)
                index = next_index;
            else if (next_lowest < max_y-slope*2 && col_diff <=2)
                index = next_index;
            else if (next_lowest < max_y-slope*3 && col_diff <=3)
                index = next_index;
            else if (next_lowest < max_y-slope*4 && col_diff <=4)
                index = next_index;
            else if (next_lowest < max_y-slope*5 && col_diff <=5)
                index = next_index;
            else if (next_lowest < max_y-slope*6 && col_diff <=6)
                index = next_index;
            else if (next_lowest < max_y-slope*7 && col_diff <=7)
                index = next_index;
            else if (next_lowest < max_y-slope*8 && col_diff <=8)
                index = next_index;
            else if (next_lowest < max_y-slope*9 && col_diff <=9)
                index = next_index;
            else if (next_lowest < max_y-slope*10 && col_diff <=10)
                index = next_index;
            else
            {
                std::cout << "2nd ball uncatchable; going for 3rd" << std::endl;
                index = high_index;
            }
        } // end of check to see if 2nd ball is catchable
    }

    int col_cmd{getColFromPixel(centers[index].x, pegs)};
    std::cout << "Sending motor to column " << col_cmd << std::endl << std::endl;

    return col_cmd;
}
