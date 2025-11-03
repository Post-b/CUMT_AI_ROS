//
// Created by Yuchen on 2025/11/3.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "square_controller");
    ros::NodeHandle nh;

    // 发布速度命令到Turtlebot2
    ros::Publisher velocityPublisher =
            nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    ros::Rate loopRate(20); // 提高到20Hz控制频率，更平滑

    // 优化的正方形参数
    double side_length = 1.5; // 边长1米（原100可能是单位问题）
    double linear_speed = 0.5; // 降低线速度到0.2 m/s，更稳定
    double angular_speed = 0.5; // 设置合理的角速度0.5 rad/s

    // 计算移动和旋转所需时间
    double move_time = side_length / linear_speed; // 移动一边所需时间
    double rotate_time = (M_PI) / angular_speed; // 旋转90度所需时间

    geometry_msgs::Twist velocityMsg;
    int sides_completed = 0;
    ros::Time start_time = ros::Time::now();

    // 添加加速和减速阶段参数
    double acceleration_time = 1.0; // 加速时间1秒
    double deceleration_time = 1.0; // 减速时间1秒

    ROS_INFO("开始正方形轨迹控制");
    ROS_INFO("边长: %.2f米, 线速度: %.2f m/s, 角速度: %.2f rad/s",
             side_length, linear_speed, angular_speed);

    while (ros::ok() && sides_completed < 4)
    {
        double elapsed_time = (ros::Time::now() - start_time).toSec();

        if (elapsed_time < move_time) {
            // 直行阶段 - 添加平滑加速减速
            if (elapsed_time < acceleration_time) {
                // 加速阶段
                velocityMsg.linear.x = linear_speed * (elapsed_time / acceleration_time);
            } else if (elapsed_time > move_time - deceleration_time) {
                // 减速阶段
                double decel_elapsed = elapsed_time - (move_time - deceleration_time);
                velocityMsg.linear.x = linear_speed * (1.0 - decel_elapsed / deceleration_time);
            } else {
                // 匀速阶段
                velocityMsg.linear.x = linear_speed;
            }
            velocityMsg.angular.z = 0.0;

            if (fmod(elapsed_time, 1.0) < 0.1) { // 每秒打印一次，避免过多输出
                ROS_INFO_STREAM("直行中... 已走距离: " << elapsed_time * linear_speed << "米");
            }
        } else if (elapsed_time < move_time + rotate_time) {
            // 旋转阶段 - 同样添加平滑处理
            double rotate_elapsed = elapsed_time - move_time;

            if (rotate_elapsed < acceleration_time) {
                // 旋转加速阶段
                velocityMsg.angular.z = angular_speed * (rotate_elapsed / acceleration_time);
            } else if (rotate_elapsed > rotate_time - deceleration_time) {
                // 旋转减速阶段
                double rotate_decel = rotate_elapsed - (rotate_time - deceleration_time);
                velocityMsg.angular.z = angular_speed * (1.0 - rotate_decel / deceleration_time);
            } else {
                // 匀速旋转
                velocityMsg.angular.z = angular_speed;
            }
            velocityMsg.linear.x = 0.0;

            if (fmod(rotate_elapsed, 0.5) < 0.1) { // 每0.5秒打印一次
                ROS_INFO_STREAM("旋转中... 已转角度: " << rotate_elapsed * angular_speed * 180/M_PI << "度");
            }
        } else {
            // 完成一边，重置计时器，准备下一边
            sides_completed++;
            start_time = ros::Time::now();
            ROS_INFO_STREAM("完成第 " << sides_completed << " 边");

            // 在开始下一段前短暂停止
            velocityMsg.linear.x = 0.0;
            velocityMsg.angular.z = 0.0;
            velocityPublisher.publish(velocityMsg);
            ros::Duration(0.5).sleep(); // 暂停0.5秒

            continue;
        }

        velocityPublisher.publish(velocityMsg);
        ros::spinOnce();
        loopRate.sleep();
    }

    // 停止机器人
    velocityMsg.linear.x = 0.0;
    velocityMsg.angular.z = 0.0;
    velocityPublisher.publish(velocityMsg);

    ROS_INFO("正方形轨迹完成！");

    return 0;
}