#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/io/vpVideoReader.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

static const std::string IMAGE_TOPIC = "/my_robot/camera1/image_raw";

vpDisplayX *d = NULL;

vpImage<unsigned char> I;
// 摄像头标定数据
vpCameraParameters cam;
// 被识别物体的位姿
vpHomogeneousMatrix cMo;

vpMbTracker *tracker;
vpMe me;
vpKltOpencv klt_settings;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gx2019_mission_control_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);
    

   
    // 等待函数
    ros::spin();
    return 0;
}

int time_ = 1;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    Mat frame;
    vpImage<unsigned char> I; // for gray images
    ROS_INFO("INSIDE.");
    try
    {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        // 第一次进入回调 ， 初始化现实界面
        if (time_ == 1)
        {
            vpImageConvert::convert(frame, I);
            d = new vpDisplayX();
            d->init(I);
            vpDisplay::setTitle(I, "visp_auto_tracker debug display");
             // 初始化配置
#if (opt_tracker == 0)
    tracker = new vpMbEdgeTracker;
#elif (opt_tracker == 1)
    tracker = new vpMbKltTracker;
#else
    tracker = new vpMbEdgeKltTracker;
#endif
    me.setMaskSize(5);
    me.setMaskNumber(180);
    me.setRange(8);
    me.setThreshold(10000);
    me.setMu1(0.5);
    me.setMu2(0.5);
    me.setSampleStep(4);
    dynamic_cast<vpMbEdgeTracker *>(tracker)->setMovingEdge(me);

    klt_settings.setMaxFeatures(300);
    klt_settings.setWindowSize(5);
    klt_settings.setQuality(0.015);
    klt_settings.setMinDistance(8);
    klt_settings.setHarrisFreeParameter(0.01);
    klt_settings.setBlockSize(3);
    klt_settings.setPyramidLevels(3);

    dynamic_cast<vpMbKltTracker *>(tracker)->setKltOpencv(klt_settings);
    dynamic_cast<vpMbKltTracker *>(tracker)->setKltMaskBorder(5);
    ROS_INFO("INSIDE.!!");

    std::string objectname = "cylinder";
    cam.initPersProjWithoutDistortion(839, 839, 325, 243);
    tracker->setCameraParameters(cam);
    tracker->loadModel(objectname + ".cao");
    tracker->setDisplayFeatures(true);
    // tracker->initClick(I, objectname + ".init", true);

            time_ += 1;
        }
        // 转换OpenCV到VISP
        vpImageConvert::convert(frame, I);
        vpDisplay::display(I);
        vpDisplay::flush(I);

        tracker->track(I);
        tracker->getPose(cMo);
        tracker->getCameraParameters(cam);
        tracker->display(I, cMo, cam, vpColor::red, 2, true);
        vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
        vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);
        vpDisplay::flush(I);

        vpTime::wait(40);
    }
    catch (const std::exception &e)
    {
    }
}