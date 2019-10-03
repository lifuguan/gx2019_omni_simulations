
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
tf::Transform qrcode_point(tf::Quaternion(0, 0, -1.0, 1), tf::Vector3(1.5, 0.8, 0));

tf::TransformListener goal_to_car_listener; tf::StampedTransform goal_to_car_stamped;

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
    ros::Rate rate(10.0);
    while (nh.ok())
    {
        goal_frame_broadcaster.sendTransform(tf::StampedTransform(qrcode_point, ros::Time::now(), "map", "goal"));

        goal_to_car_listener.lookupTransform("/base_link", "/goal", ros::Time(0), goal_to_car_stamped);
        ROS_INFO("%d %d ", goal_to_car_stamped.getOrigin().x(), goal_to_car_stamped.getOrigin().y());
        ros::spinOnce();
    }

    return 0;
}
