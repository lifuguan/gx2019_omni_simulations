/*
 * @Description: 用于测试小车底盘
 * @Author: your name
 * @Date: 2019-10-19 14:19:17
 * @LastEditTime: 2019-10-29 21:28:47
 * @LastEditors: Please set LastEditors
 */
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

using namespace std;
geometry_msgs::Twist cmd_vel;

tf::Transform test_point(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1.2, 0.8, 0));

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gx2019_chassis_test");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    tf::TransformBroadcaster goal_frame_broadcaster;
    tf::TransformListener goal_to_car_listener;
    tf::StampedTransform goal_to_car_stamped;

    ros::Rate rate(10.0);
    while (nh.ok())
    {

        goal_frame_broadcaster.sendTransform(tf::StampedTransform(test_point, ros::Time::now(), "goal", "map"));

        //避免第一次订阅时翻车
        try
        {
            goal_to_car_listener.lookupTransform("base_link", "goal", ros::Time(0), goal_to_car_stamped);

            //四元数转RPY ， 使用yaw
            double yaw, pitch, roll;
            tf::Matrix3x3(goal_to_car_stamped.getRotation()).getEulerYPR(yaw, pitch, roll);

            double x, y, rotation, dst;
            y = goal_to_car_stamped.getOrigin().y();
            x = goal_to_car_stamped.getOrigin().x();
            ROS_INFO("vel: %f %f", x, y);

            rotation = yaw;
            dst = sqrt(pow(x, 2) + pow(y, 2));
            if (dst <= 0.1)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                ROS_INFO("vel: %f %f", cmd_vel.linear.x, cmd_vel.linear.y);

                pub.publish(cmd_vel);
            }
            else
            {
                cmd_vel.linear.x = 0.6 * tanh(dst) * (x / dst);
                cmd_vel.linear.y = 0.6 * tanh(dst) * (y / dst);
            }
            pub.publish(cmd_vel);
            ROS_INFO("%f", dst);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
        }

        rate.sleep();
    }

    return 0;
}