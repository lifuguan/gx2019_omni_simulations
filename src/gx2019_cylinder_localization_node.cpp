/*
 * @Description: 色块识别和追踪
 * @Author: lifuguan
 * @Date: 2019-10-03 15:21:38
 * @LastEditTime: 2019-11-02 23:44:13
 * @LastEditors: Please set LastEditors
 */
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gx2019_omni_simulations/arm_transport.h>
#include <gx2019_omni_simulations/cv_mission_type.h>

using namespace std;
using namespace cv;

static const std::string IMAGE_TOPIC = "/usb_cam/image_raw";

//设置颜色区间
Scalar red_range[3] = {Scalar(0, 160, 50), Scalar(10, 255, 255)};
Scalar green_range[3] = {Scalar(25, 43, 46), Scalar(95, 255, 255)};
Scalar blue_range[3] = {Scalar(98, 102, 20), Scalar(130, 255, 255)};

ros::Publisher cv_material_queue_pub;
ros::Publisher arm_transport_pub;

gx2019_omni_simulations::arm_transport arm_transport;

/* cv的任务类型
    0：啥也不干
    1：识别色块排列
    2：追踪指定色块
    3：发送”抓”指令
*/
int cv_mission_type = 0;

int target_color = 233;
enum TARGET_COLOR
{
    red,
    blue,
    green
};

// 获取图像的回调函数
void imageCallback(const sensor_msgs::ImageConstPtr &msg);
// 获取视觉任务的回调函数
void cvMissionCallBack(const gx2019_omni_simulations::cv_mission_type &msg);
// 初始化图形
string detect(vector<Point> cnts_single, vector<Point> &approx);
// 圈出图形
void drawLine(Mat &frame, string shape, vector<Point> &approx);
// 找到色块的质心
int findMaterialCentroid(Mat target_frame);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gx2019_cylinder_localization_node");
    ros::NodeHandle nh;

    ros::Subscriber cv_mission_sub = nh.subscribe("cv_mission_topic", 10, cvMissionCallBack);
    cv_material_queue_pub = nh.advertise<gx2019_omni_simulations::cv_mission_type>("cv_material_queue", 10);
    arm_transport_pub = nh.advertise<gx2019_omni_simulations::arm_transport>("arm_transport", 10);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);

    namedWindow("OPENCV_WINDOW");
    ros::Rate loop(5);
    int time = 1;
    while (nh.ok())
    {
        double pixel_ = 0;
        if (cX_ - (640 - 279.9) < 0)
        {
            pixel_ = cX_ - (640 - 279.9);
        }
        else
        {
            pixel_ = cX_ - (640 - 360.1);
        }
        double angular = atan(pixel_ / 651.3) * 360 / (2 * M_PI);
        if (cX_ == 320)
        {
        }
        // else if (abs(angular) <= 0.5)
        // {
        //     arm_transport.arm_moveit = true;
        //     arm_transport.gimbal_rotate = 0;
        //     arm_transport_pub.publish(arm_transport);
        // }
        else
        {
            if (time == 1)
            {
                cout << "angular : " << angular << endl;
                arm_transport.arm_moveit = false;
                arm_transport.gimbal_rotate = angular;
                arm_transport_pub.publish(arm_transport);

                arm_transport.arm_moveit = true;
                arm_transport.gimbal_rotate = 0;
                arm_transport_pub.publish(arm_transport);
                time += 1;
            }
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

void cvMissionCallBack(const gx2019_omni_simulations::cv_mission_type &msg)
{
    cv_mission_type = msg.cv_mission_type;
    target_color = msg.cv_target_color;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // 直接退出程序
    if (cv_mission_type == 0)
    {
        return;
    }
    else if (cv_mission_type == 3)
    {
        arm_transport.arm_moveit = true;
        arm_transport.gimbal_rotate = 0;
        arm_transport_pub.publish(arm_transport);
    }

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

    // 识别物料区中色块的排列色块
    if (cv_mission_type == 1)
    {
        // 图像格式切换
        cvtColor(frame, frame_hsv, CV_BGR2HSV);

        Mat red_frame, blue_frame, green_frame;

        inRange(frame_hsv, red_range[0], red_range[1], red_frame);
        inRange(frame_hsv, blue_range[0], blue_range[1], blue_frame);
        inRange(frame_hsv, green_range[0], green_range[1], green_frame);

        imshow("OPENCV_WINDOW", red_frame | blue_frame | green_frame);

        // 获取不同颜色块的质点位置
        int red_cX = findMaterialCentroid(red_frame);
        int blue_cX = findMaterialCentroid(blue_frame);
        int green_cX = findMaterialCentroid(green_frame);

        gx2019_omni_simulations::cv_mission_type cv_material_queue;
        // 排序
        if (red_cX < blue_cX)
        {
            if (blue_cX < green_cX) // red < blue < green
            {
                cv_material_queue.material_queue = {0, 1, 2};
            }
            else if (green_cX < red_cX) // green < red < blue
            {
                cv_material_queue.material_queue = {2, 0, 1};
            }
            else // red < green < blue
            {
                cv_material_queue.material_queue = {0, 2, 1};
            }
        }
        else if (blue_cX < red_cX)
        {
            if (red_cX < green_cX) // blue < red < green
            {
                cv_material_queue.material_queue = {1, 0, 2};
            }
            else if (green_cX < blue_cX) // green < blue < red
            {
                cv_material_queue.material_queue = {2, 1, 0};
            }
            else // blue < green < red
            {
                cv_material_queue.material_queue = {1, 2, 0};
            }
        }
        cv_material_queue_pub.publish(cv_material_queue);
    }
    // 抓取指定色块
    else if (cv_mission_type == 2)
    {
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
            if (cnts_single.size() > 50)
            {
                vector<Point> approx;
                string shape = detect(cnts_single, approx);
                if (shape == "rectangle" || shape == "sqaure")
                {
                    Moments M = moments(cnts_single);
                    int cX, cY;
                    if (M.m10 != 0)
                    {
                        //表示图像重心
                        cX = int((M.m10 / M.m00));
                        cY = int((M.m01 / M.m00));
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
                    imshow("OPENCV_WINDOW", frame);

                    arm_transport.arm_moveit = false;
                    arm_transport.gimbal_rotate = cX;
                    arm_transport_pub.publish(arm_transport);
                }
            }
            else
            {
                //printf("too fucking small %d\n", cnts_single.size());
            }
        }
    }
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

int findMaterialCentroid(Mat target_frame)
{
    int cX;
    vector<vector<Point>> cnts; //获取了一堆又一堆点
    findContours(target_frame, cnts, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    vector<Point> cnts_single = cnts[0]; //获取了上面一堆点中的一个点
    if (cnts_single.size() > 50)
    {
        vector<Point> approx;
        string shape = detect(cnts_single, approx);
        Moments M = moments(cnts_single);

        if (M.m10 != 0)
        {
            //表示图像重心
            cX = int((M.m10 / M.m00));
        }
        else
        {
            cX = 0;
        }
    }
    return cX;
}