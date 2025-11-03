#include <ros/ros.h>
<<<<<<< HEAD
#include <std_msgs/String.h>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    ros::Publisher chatterPublisher =
        nh.advertise<std_msgs::String>("chatter", 1);
    ros::Rate loopRate(10);

    unsigned int count = 0;
    while (ros::ok())
    {
        std_msgs::String message;
        message.data = "hello world " + std::to_string(count);
        ROS_INFO_STREAM(message.data);
        chatterPublisher.publish(message);
        ros::spinOnce();
        loopRate.sleep();
        count++;
    }
    return 0;
}
=======
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include <cmath>

class YawComparator
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_raw_sub_;
    ros::Subscriber imu_sub_;
    
    // 积分法相关变量
    double last_time_;
    double theta_1_;  // 通过积分得到的yaw角度
    bool first_raw_message_;
    
    // 四元数法相关变量
    double theta_3_;  // 通过四元数得到的yaw角度（局部坐标系）
    bool first_imu_message_;
    
    // 初始姿态四元数
    double init_w_, init_x_, init_y_, init_z_;
    
public:
    YawComparator() : theta_1_(0.0), theta_3_(0.0), first_raw_message_(true), first_imu_message_(true),
                     init_w_(0.0), init_x_(0.0), init_y_(0.0), init_z_(0.0)
    {
        imu_raw_sub_ = nh_.subscribe("/mobile_base/sensors/imu_data_raw", 10, 
                                   &YawComparator::imuRawCallback, this);
        imu_sub_ = nh_.subscribe("/mobile_base/sensors/imu_data", 10, 
                               &YawComparator::imuCallback, this);
        ROS_INFO("Yaw comparator initialized, waiting for IMU data...");
    }
    
    void imuRawCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // 获取当前时间戳
        double current_time = msg->header.stamp.toSec();
        
        if (first_raw_message_) {
            // 第一次收到消息，初始化时间戳
            last_time_ = current_time;
            first_raw_message_ = false;
            ROS_INFO("First IMU raw message received, initializing theta_1 to 0");
            return;
        }
        
        // 计算时间间隔
        double dt = current_time - last_time_;
        
        // 获取z轴角速度（yaw轴）
        double angular_velocity_z = msg->angular_velocity.z;
        
        // 使用积分公式：θ_1 = θ_1 + dt * ω
        theta_1_ += dt * angular_velocity_z;
        
        // 更新上一次的时间戳
        last_time_ = current_time;
        
        // 输出比较结果
        printComparison();
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // 获取当前四元数
        double curr_w = msg->orientation.w;
        double curr_x = msg->orientation.x;
        double curr_y = msg->orientation.y;
        double curr_z = msg->orientation.z;
        
        if (first_imu_message_) {
            // 第一次收到IMU消息，记录初始姿态
            init_w_ = curr_w;
            init_x_ = curr_x;
            init_y_ = curr_y;
            init_z_ = curr_z;
            first_imu_message_ = false;
            theta_3_ = 0.0;  // 初始化局部yaw角为0
            ROS_INFO("First IMU message received, initial orientation set.");
            return;
        }
        
        // 计算相对旋转：从初始姿态到当前姿态的旋转
        // 相对四元数 = 初始四元数的共轭 × 当前四元数
        double rel_w, rel_x, rel_y, rel_z;
        
        // 计算初始四元数的共轭（共轭四元数的w相同，x,y,z取反）
        double init_conj_w = init_w_;
        double init_conj_x = -init_x_;
        double init_conj_y = -init_y_;
        double init_conj_z = -init_z_;
        
        // 四元数乘法：共轭四元数 × 当前四元数
        rel_w = init_conj_w * curr_w - init_conj_x * curr_x - init_conj_y * curr_y - init_conj_z * curr_z;
        rel_x = init_conj_w * curr_x + init_conj_x * curr_w + init_conj_y * curr_z - init_conj_z * curr_y;
        rel_y = init_conj_w * curr_y - init_conj_x * curr_z + init_conj_y * curr_w + init_conj_z * curr_x;
        rel_z = init_conj_w * curr_z + init_conj_x * curr_y - init_conj_y * curr_x + init_conj_z * curr_w;
        
        // 从相对旋转四元数中提取yaw角度（局部坐标系）
        extractYawFromQuaternion(rel_w, rel_x, rel_y, rel_z, theta_3_);
        
        // 如果已经收到过raw数据，则输出比较结果
        if (!first_raw_message_) {
            printComparison();
        }
    }
    
    void extractYawFromQuaternion(double w, double x, double y, double z, double& yaw)
    {
        // 从四元数中提取yaw角度
        // 使用公式: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
        
        // 确保yaw角在[-π, π]范围内
        normalizeAngle(yaw);
    }
    
    void normalizeAngle(double& angle)
    {
        // 将角度归一化到[-π, π]范围
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
    }
    
    void printComparison()
    {
        // 转换为角度值便于比较
        double theta_1_deg = theta_1_ * 180.0 / M_PI;
        double theta_3_deg = theta_3_ * 180.0 / M_PI;
        
        // 计算两种方法的差异
        double diff_rad = theta_1_ - theta_3_;
        normalizeAngle(diff_rad);  // 归一化差异角度
        
        double diff_deg = diff_rad * 180.0 / M_PI;
        
        ROS_INFO("=== Yaw Angle Comparison ===");
        ROS_INFO("Integral method (θ_1): %8.4f rad, %8.4f deg", theta_1_, theta_1_deg);
        ROS_INFO("Quaternion method (θ_3): %8.4f rad, %8.4f deg", theta_3_, theta_3_deg);
        ROS_INFO("Difference (θ_1 - θ_3): %8.4f rad, %8.4f deg", diff_rad, diff_deg);
        ROS_INFO("============================");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yaw_comparator");
    
    YawComparator yaw_comparator;
    
    ros::spin();
    
    return 0;
}
>>>>>>> a4f57f97a98e3e8bdc656915a38a351b20bddd86
