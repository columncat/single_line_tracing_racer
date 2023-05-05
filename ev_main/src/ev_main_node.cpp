#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "ev_main/MotorMsg.h"
#include "ev_main/ServoMsg.h"

using namespace cv;
using namespace std;

/* for initiating camera */
std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

/* Cropping frames */
Mat reg_of_int(Mat img)
{
    int img_h = img.rows;
    int img_w = img.cols;
    double padding = 0.05;

    Point points_exclude[] = {
        Point((int)(padding * img_w), (int)(padding * img_h)),
        Point((int)(padding * img_w), (int)((1 - padding) * img_h)),
        Point((int)((1 - padding) * img_w), (int)((1 - padding) * img_h)),
        Point((int)((1 - padding) * img_w), (int)(padding * img_h))};

    const Point *ppt_exclude[1] = {points_exclude};
    int npt[] = {4};

    Mat mask(img.size(), img.type(), Scalar(255, 255, 255));
    fillPoly(mask, ppt_exclude, npt, 1, Scalar(0, 0, 0), LINE_8);

    // Apply Mask to the Image
    Mat masked_img;
    bitwise_and(img, mask, masked_img);

    return masked_img;
}

/* Read camera and return x, x2, theta */
/* x for line location of vehicle, x2 for line location in front of vehicle */
void get_disp(const Mat &masked, int *x, int &theta, int &num_detected)
{
    std::vector<std::vector<Point>> contours;
    findContours(masked, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    num_detected = 0;
    Point potents[2];
    for (size_t i = 0; i < contours.size(); i++)
    {
        // there should be 2 contours in general
        if (contourArea(contours[i]) > 50)
        {
            Rect bounding_rect = boundingRect(contours[i]);
            Point center(bounding_rect.x + bounding_rect.width / 2, bounding_rect.y + bounding_rect.height / 2);
            if (num_detected < 2)
                potents[num_detected++] = center;
        }
    }
    // do not update value for edge cases
    if (num_detected != 2)
        return;
    if (potents[0].y == potents[1].y)
        return;

    int holder = potents[0].y > potents[1].y ? 0 : 1;
    int follower = !holder;
    x[0] = potents[holder].x - masked.cols / 2;
    x[1] = potents[follower].x - masked.cols / 2;

    double radian = atan2(potents[holder].x - potents[follower].x, potents[holder].y - potents[follower].y);
    theta = (int)(radian * 180 / M_PI);

    // setting range of atan2 to -Pi/2 to PI/2
    if (theta > 90)
        theta = theta - 180;
    if (theta < -90)
        theta = theta + 180;
    return;
}

double sigmoid(double x)
{
    return (1 / (1 + pow(2.71, -x)));
}

double get_factor(double dx, double min, double amp)
{
    // max = min + amp
    return min + amp * sigmoid(dx);
}

void get_steer(int *x, int theta, int &steering_angle)
{
    double dx = (double)(abs(x[1]) - abs(x[0])) / 140;
    double xavg = (double)(x[0] + x[1]) / 2;

    /* increase factor of theta when line goes out of sight
       decrease factor of theta when line crosses across the sight
       follow xavg when xavg is big enough */
    steering_angle = theta * get_factor(dx, 1.5, 0.4) + pow((abs(xavg) / 320), 2) * xavg / abs(xavg) * -45;

    steering_angle = steering_angle > 45 ? 45 : steering_angle;
    steering_angle = steering_angle < -45 ? -45 : steering_angle;
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ev_main_node");
    ros::NodeHandle nh;

    ros::Publisher motor_pub = nh.advertise<ev_main::MotorMsg>("motor", 10);
    ros::Publisher servo_pub = nh.advertise<ev_main::ServoMsg>("servo", 10);

    ros::Rate loop_rate(100); // unit in Hz

    ev_main::MotorMsg motorMsg;
    ev_main::ServoMsg servoMsg;

    int cam_width = 320;
    int cam_height = 240;
    int display_width = 320;
    int display_height = 240;
    int framerate = 100;
    int flip_method = 0;

    std::string pipeline = gstreamer_pipeline(cam_width,
                                              cam_height,
                                              display_width,
                                              display_height,
                                              framerate,
                                              flip_method);

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }

    // set range of yellow in hsv scale (NOT rgb)
    Scalar lower_yellow(20, 51, 85);
    Scalar upper_yellow(70, 255, 255);

    int x[2] = {0, 0}, theta = 0, num_detected = 0, steering_angle = 0;

    motorMsg.data = "a";
    motor_pub.publish(motorMsg);

    while (ros::ok())
    {
        Mat frame;
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "Failed to capture frame" << std::endl;
            break;
        }

        Mat roi;
        roi = reg_of_int(frame);

        Mat hsv;
        cvtColor(roi, hsv, COLOR_BGR2HSV);
        Mat yellow_mask;
        inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

        get_disp(yellow_mask, x, theta, num_detected); // num_detected was used for debugging
        get_steer2(x, theta, steering_angle);

        ROS_INFO("%d, %d, %d, %d, %d", num_detected, x[0], x[1], theta, steering_angle);

        char res[4];
        res[0] = steering_angle > 0 ? 'l' : 'r';
        res[1] = '0' + (abs(steering_angle) / 10);
        res[2] = '0' + (abs(steering_angle) % 10);
        servoMsg.data = res;
        servo_pub.publish(servoMsg);

        loop_rate.sleep();
    }
    cap.release();
    return 0;
}

// source ~/catkin_ws/devel/setup.bash
