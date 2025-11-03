//
// Created by Yuchen on 2025/11/3.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int frame_count = 0;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    frame_count++;
    // 直接用 PointCloud2 的宽高计算点数
    unsigned int point_count = msg->width * msg->height;
    //frame是帧数，points是每帧点云个数
    ROS_INFO("Frame: %d | Points: %u", frame_count, point_count);
}

int main(int argc, char** argv)
{
    //订阅的是/livox/lidar这个话题
    ros::init(argc, argv, "livox_listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/livox/lidar", 10, pointCloudCallback);

    ROS_INFO("Listening to /livox/lidar ...");

    ros::spin();
    return 0;
}
