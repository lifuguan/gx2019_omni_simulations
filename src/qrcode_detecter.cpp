/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-10-03 15:21:38
 * @LastEditTime: 2019-10-18 20:22:39
 * @LastEditors: Please set LastEditors
 */
 #include <iostream>

#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpPose.h>

#include <stdlib.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
static const std::string IMAGE_TOPIC = "/usb_cam/image_raw";
Mat frame;

vpDisplayX *d = NULL;
vpImage<unsigned char> I; // for gray images
vpHomogeneousMatrix cMo;
bool init = true;
vpDetectorQRCode detector;
// Camera parameters should be adapted to your camera
vpCameraParameters cam(840, 840, I.getWidth() / 2, I.getHeight() / 2);

// 3D model of the QRcode: here we consider a 12cm by 12cm QRcode
std::vector<vpPoint> point;

ros::Publisher pub;

bool abort_control = false;

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam,
                 bool init, vpHomogeneousMatrix &cMo);

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qrcode_detecter");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);
  pub = nh.advertise<std_msgs::String>("qrcode_message", 10);
  try
  {
    point.push_back(vpPoint(-0.06, -0.06,
                            0)); // QCcode point 0 3D coordinates in plane Z=0
    point.push_back(vpPoint(0.06, -0.06,
                            0));             // QCcode point 1 3D coordinates in plane Z=0
    point.push_back(vpPoint(0.06, 0.06, 0)); // QCcode point 2 3D coordinates in plane Z=0
    point.push_back(vpPoint(-0.06, 0.06,
                            0)); // QCcode point 3 3D coordinates in plane Z=0
  }
  catch (const vpException &e)
  {
    // std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
  while (nh.ok())
  {
    // 如果任务已被获取，则结束该节点
    if (abort_control == true)
    {
      return 0;
    }
    else
    {
      ros::spinOnce();
    }
  }
  return 0;
}
int time_ = 1;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {

    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    // 第一次进入回调 ， 初始化现实界面
    if (time_ == 1)
    {
      vpImageConvert::convert(frame, I);
      time_ += 1;
    }

    // 从opencv Mat格式转换到 VISP格式
    vpImageConvert::convert(frame, I);

    bool status = detector.detect(I);

    if (status)
    { // true if at least one QRcode is detected
      for (size_t i = 0; i < detector.getNbObjects(); i++)
      {

        std::vector<vpImagePoint> p = detector.getPolygon(i); // get the four corners location in the image

        // 信息读取
        vpRect bbox = detector.getBBox(i);
        std_msgs::String msg;
        msg.data = detector.getMessage(i);
        cout << msg.data << endl;
        pub.publish(msg);
        abort_control = true;

        // 位姿读取
        computePose(point, p, cam, init,
                    cMo); // resulting pose is available in cMo var
      }
    }
    vpImageConvert::convert(I, frame);
    vpTime::wait(40);
  }
  catch (cv_bridge::Exception &e)
  {
    //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam,
                 bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;
  double x = 0, y = 0;
  for (unsigned int i = 0; i < point.size(); i++)
  {
    vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true)
  {
    vpHomogeneousMatrix cMo_dem;
    vpHomogeneousMatrix cMo_lag;
    pose.computePose(vpPose::DEMENTHON, cMo_dem);
    pose.computePose(vpPose::LAGRANGE, cMo_lag);
    double residual_dem = pose.computeResidual(cMo_dem);
    double residual_lag = pose.computeResidual(cMo_lag);
    if (residual_dem < residual_lag)
      cMo = cMo_dem;
    else
      cMo = cMo_lag;
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);
}