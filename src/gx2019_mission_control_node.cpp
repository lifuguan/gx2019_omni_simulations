
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

tf::Transform goals[2][4] =
    {
        {tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.86, -2.0, 0)),
         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1.05, -2.0, 0)),
         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1.25, -2.0, 0))},
        {tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-2.0, -1.36, 0)),
         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-2.0, -1.51, 0)),
         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-2.0, -1.68, 0))}};

tf::Transform inital_point(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.2, -0.2, 0));
tf::Transform qrcode_point(tf::Quaternion(0, 0, 1.0, 1), tf::Vector3(0.8, -1.5, 0));

int qrcode_message[2][4];

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

    tf::TransformBroadcaster goal_frame_broadcaster;
    tf::TransformListener goal_to_car_listener;
    tf::StampedTransform goal_to_car_stamped;

    ros::Rate rate(1.0);
    while (nh.ok())
    {

        goal_frame_broadcaster.sendTransform(tf::StampedTransform(qrcode_point, ros::Time::now(), "goal", "map"));

        try
        {

            goal_to_car_listener.lookupTransform("goal", "base_link", ros::Time(0), goal_to_car_stamped);

            //四元数转RPY ， 使用yaw
            double yaw, pitch, roll;
            tf::Matrix3x3(goal_to_car_stamped.getRotation()).getEulerYPR(yaw, pitch, roll);

            double x, y, rotation;
            x = -goal_to_car_stamped.getOrigin().y();  y = goal_to_car_stamped.getOrigin().x();  rotation = yaw;
            
            ROS_INFO("%f %f %f", x, y, rotation);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            // vel_msg.linear.x = 0;
            // vel_msg.angular.z = 0;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
