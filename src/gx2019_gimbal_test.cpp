/*
 * @Description: In User Settings Edit
 * @Author: lifuguan
 * @Date: 2019-10-03 15:21:38
 * @LastEditTime: 2019-10-25 22:56:50
 * @LastEditors: Please set LastEditors
 */
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gx2019_omni_simulations/arm_transport.h>

using namespace std;
using namespace cv;

static const std::string IMAGE_TOPIC = "/usb_cam/image_raw";

//设置颜色区间
Scalar red_range[3] = {Scalar(0, 160, 50), Scalar(10, 255, 255)};
Scalar green_range[3] = {Scalar(25, 189, 118), Scalar(95, 255, 198)};
Scalar blue_range[3] = {Scalar(98, 102, 20), Scalar(130, 255, 255)};

int target_color = 0;
enum TARGET_COLOR
{
    red,
    blue,
    green
};
ros::Publisher arm_transport_pub;
gx2019_omni_simulations::arm_transport arm_transport;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
//初始化图形并圈出
string detect(vector<Point> cnts_single, vector<Point> &approx);

void drawLine(Mat &frame, string shape, vector<Point> &approx);

int cX_ = 320;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gx2019_gimbal_test");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 10, imageCallback);
    arm_transport_pub = nh.advertise<gx2019_omni_simulations::arm_transport>("arm_transport", 1);
    namedWindow("OPENCV_WINDOW");
    ros::Rate loop(5);
    while (nh.ok())
    {
        //cX_ += 1;
        if (abs(cX_ - 320) <= 10 && abs(cX_ - 320) >= 1)
        {
            arm_transport.arm_moveit = true;
            arm_transport.gimbal_rotate = 320;
        }
        else
        {
            arm_transport.arm_moveit = false;
            arm_transport.gimbal_rotate = cX_;
        }
        arm_transport_pub.publish(arm_transport);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    Mat frame_hsv, frame;
    try
    {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("%s", e.what());
        return;
    }

    //图像格式切换
    cvtColor(frame, frame_hsv, COLOR_BGR2HSV);

    Mat target_frame;
    if (target_color == red)
    {
        inRange(frame_hsv, red_range[0], red_range[1], target_frame);
    }
    else if (target_color == blue)
    {
        inRange(frame_hsv, blue_range[0], blue_range[1], target_frame);
    }
    else
    {
        inRange(frame_hsv, green_range[0], green_range[1], target_frame);
    }

    //
    Mat erode_kernel = getStructuringElement(MORPH_RECT, Size(2, 2));
    erode(target_frame, target_frame, erode_kernel, Point(-1, -1), 1);
    //
    Mat dilate_kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(target_frame, target_frame, dilate_kernel, Point(-1, -1), 1);

    vector<vector<Point>> cnts; //获取了一堆又一堆点
    findContours(target_frame, cnts, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    for (int i = 0; i < cnts.size(); i++)
    {
        vector<Point> cnts_single = cnts[i]; //获取了上面一堆点中的一个点
        if (cnts_single.size() > 200)
        {
            vector<Point> approx;
            string shape = detect(cnts_single, approx);
            Moments M = moments(cnts_single);
            int cX, cY;
            if (M.m10 != 0)
            {
                //表示图像重心
                cX = int((M.m10 / M.m00));
                cY = int((M.m01 / M.m00));
                cX_ = cX;
            }
            else
            {
                cX = cY = 0;
            }
            //画中位线
            line(frame, Point(frame.cols / 2, 0), Point(frame.cols / 2, frame.rows), (255, 255, 255), 2);
            //画质心线
            line(frame, Point(cX, cY), Point(0, cY), Scalar(255, 255, 255), 1);
            line(frame, Point(cX, cY), Point(cX, 0), Scalar(255, 255, 255), 1);

            putText(frame, "black line " + shape, Point(cX, cY), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
            line(frame, Point(cX, cY), Point(cX, cY), (255, 255, 255), 5);
            putText(frame, "position " + to_string(cX) + " , " + to_string(cY), Point(cX, cY + 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255), 1); //质心位置
            drawLine(frame, shape, approx);                                                                                                             //画矩形
        }
        else
        {
            //printf("too fucking small %d\n", cnts_single.size());
        }
    }
    imshow("OPENCV_WINDOW", frame);

    waitKey(30);
}

//初始化图形并圈出
string detect(vector<Point> cnts_single, vector<Point> &approx)
{
    string shape = "undentified";
    double peri = arcLength(cnts_single, true);
    approxPolyDP(cnts_single, approx, 0.045 * peri, true);
    if (approx.size() == 3)
    {
        shape = "triangle";
    }
    else if (approx.size() == 4)
    {
        shape = "rectangle";
        /*待完成*/
    }
    else if (approx.size() == 5)
    {
        shape = "pentagon";
    }
    else
    {
        shape = "circle";
    }

    return shape;
}

void drawLine(Mat &frame, string shape, vector<Point> &approx)
{
    Point pt[13];
    if (shape == "rectangle" || shape == "sqaure")
    {
        pt[1] = approx[0];
        pt[2] = approx[1];
        pt[3] = approx[2];
        pt[4] = approx[3];
        line(frame, pt[1], pt[2], Scalar(0, 255, 0), 2);
        line(frame, pt[3], pt[2], Scalar(0, 255, 0), 2);
        line(frame, pt[4], pt[3], Scalar(0, 255, 0), 2);
        line(frame, pt[4], pt[1], Scalar(0, 255, 0), 2);
    }
    else if (shape == "pentagon")
    {
        pt[1] = approx[0];
        pt[2] = approx[1];
        pt[3] = approx[2];
        pt[4] = approx[3];
        pt[5] = approx[4];
        line(frame, pt[1], pt[2], Scalar(0, 255, 0), 3);
        line(frame, pt[3], pt[2], Scalar(0, 255, 0), 3);
        line(frame, pt[4], pt[3], Scalar(0, 255, 0), 3);
        line(frame, pt[4], pt[5], Scalar(0, 255, 0), 3);
        line(frame, pt[5], pt[1], Scalar(0, 255, 0), 3);
    }
    else if (shape == "cross")
    {
        pt[1] = approx[0];
        pt[2] = approx[1];
        pt[3] = approx[2];
        pt[4] = approx[3];
        pt[5] = approx[4];
        pt[6] = approx[5];
        line(frame, pt[1], pt[2], Scalar(0, 255, 0), 3);
        line(frame, pt[3], pt[2], Scalar(0, 255, 0), 3);
        line(frame, pt[4], pt[3], Scalar(0, 255, 0), 3);
        line(frame, pt[4], pt[5], Scalar(0, 255, 0), 3);
        line(frame, pt[5], pt[6], Scalar(0, 255, 0), 3);
        line(frame, pt[1], pt[1], Scalar(0, 255, 0), 3);
    }
}
