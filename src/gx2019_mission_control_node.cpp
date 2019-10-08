/*
 * @Description: 负责收取qrcode的信息， 设置导航目标点
 * @Author: lifuguan
 * @Date: 2019-10-03 17:21:04
 * @LastEditTime: 2019-10-08 20:07:08
 * @LastEditors: Please set LastEditors
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

tf::Transform goals[2][4] =
    {
        {tf::Transform(tf::Quaternion(0, 0, 1.0, 1), tf::Vector3(2.0, -0.86, 0)),
         tf::Transform(tf::Quaternion(0, 0, 1.0, 1), tf::Vector3(2.0, -1.05, 0)),
         tf::Transform(tf::Quaternion(0, 0, 1.0, 1), tf::Vector3(2.0, -1.25, 0))},
        {tf::Transform(tf::Quaternion(0, 0, 1.0, 1), tf::Vector3(1.36, -2.0, 0)),
         tf::Transform(tf::Quaternion(0, 0, 1.0, 1), tf::Vector3(1.51, -2.0, 0)),
         tf::Transform(tf::Quaternion(0, 0, 1.0, 1), tf::Vector3(1.68, -2.0, 0))}};

tf::Transform inital_point(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.2, -0.2, 0));
tf::Transform qrcode_point(tf::Quaternion(0, 0, 1.0, 1), tf::Vector3(0.8, -1.8, 0));

int qrcode_message[2][4] = {0};

void qrcode_message_callback(const std_msgs::String qrcode_message_)
{
    for (int i = 0; i < 3; i++)
    {
        qrcode_message[0][i] = qrcode_message_.data[i] - 48;
        qrcode_message[1][i] = qrcode_message_.data[4 + i] - 48;
    }
    ROS_INFO("%d %d %d \n %d %d %d", qrcode_message[0][0], qrcode_message[0][1], qrcode_message[0][2], qrcode_message[1][0], qrcode_message[1][1], qrcode_message[1][2]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gx2019_mission_control_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("qrcode_message", 10, qrcode_message_callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    tf::TransformBroadcaster goal_frame_broadcaster;
    tf::TransformListener goal_to_car_listener;
    tf::StampedTransform goal_to_car_stamped;

    geometry_msgs::Twist cmd_vel;

    ros::Rate rate(5.0);
    while (nh.ok())
    {
        if (qrcode_message[0][0] == 0)
        {
            goal_frame_broadcaster.sendTransform(tf::StampedTransform(qrcode_point, ros::Time::now(), "goal", "map"));
        }
        else
        {
            goal_frame_broadcaster.sendTransform(tf::StampedTransform(goals[0][0], ros::Time::now(), "goal", "map"));
        }
        
        try
        {
            
            goal_to_car_listener.lookupTransform("goal", "base_link", ros::Time(0), goal_to_car_stamped);

            //四元数转RPY ， 使用yaw
            double yaw, pitch, roll;
            tf::Matrix3x3(goal_to_car_stamped.getRotation()).getEulerYPR(yaw, pitch, roll);

            double x, y, rotation, dst;
            x = -goal_to_car_stamped.getOrigin().y();
            y = goal_to_car_stamped.getOrigin().x();
            rotation = yaw;
            dst = sqrt(pow(x, 2) + pow(y, 2));
            if (dst <= 0.02)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
            }
            else
            {
                cmd_vel.linear.x = 0.3 * tanh(dst) * (x / dst);
                cmd_vel.linear.y = 0.3 * tanh(dst) * (y / dst);
                // cmd_vel.angular.z = -0.7 * rotation;
                // cmd_vel.angular.z = atan2(x, y);
            }
            ROS_INFO("%f %f %f", x, y, rotation);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            // vel_msg.angular.z = 0;
        }

        pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
