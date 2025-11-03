////
//// Created by Yuchen on 2025/11/3.
////


#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <stdlib.h>

class UWBTagParser {
private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    int serial_fd_;
    bool serial_connected_;

    static const int FRAME_LENGTH = 22; // 修正为22字节：2(x)+2(y)+2(yaw)+12(6个距离)+2(错误状态)+2(z)

public:
    UWBTagParser() : serial_fd_(-1), serial_connected_(false) {
        if (!ros::master::check()) {
            ROS_ERROR("Cannot connect to ROS Master! Please run roscore first.");
            return;
        }

        ROS_INFO("Connected to ROS Master");
        pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("uwb_pose", 10);

        if (!connectSerial()) {
            ROS_ERROR("Serial connection failed");
            return;
        }

        serial_connected_ = true;
        ROS_INFO("UWB Tag Parser initialized successfully");
    }

    bool connectSerial() {
        std::string port;
        nh_.param<std::string>("serial_port", port, "/dev/ttyUSB0");

        ROS_INFO("Trying to open serial port: %s", port.c_str());

        // 检查串口设备是否存在
        if (access(port.c_str(), F_OK) == -1) {
            ROS_WARN("Serial device not found: %s", port.c_str());

            // 尝试自动检测串口设备
            std::vector<std::string> possible_ports = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"};
            bool found = false;
            for (const auto& p : possible_ports) {
                if (access(p.c_str(), F_OK) != -1) {
                    port = p;
                    ROS_INFO("Found serial device: %s", port.c_str());
                    found = true;
                    break;
                }
            }

            if (!found) {
                ROS_ERROR("No serial devices found. Please check connection.");
                return false;
            }
        }

        // 检查权限
        if (access(port.c_str(), R_OK | W_OK) == -1) {
            ROS_ERROR("No permission to access %s. Please run:", port.c_str());
            ROS_ERROR("sudo chmod 666 %s", port.c_str());
            ROS_ERROR("Or add your user to dialout group: sudo usermod -a -G dialout $USER");
            return false;
        }

        // 打开串口
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            ROS_ERROR("Failed to open serial port %s: %s", port.c_str(), strerror(errno));
            return false;
        }

        // 配置串口
        struct termios tty;
        memset(&tty, 0, sizeof(tty));

        if (tcgetattr(serial_fd_, &tty) != 0) {
            ROS_ERROR("Failed to get serial attributes: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // 设置波特率
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        // 设置串口参数
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 data bits
        tty.c_iflag &= ~IGNBRK; // dont ignore break
        tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
        tty.c_oflag = 0; // no remapping, no delays
        tty.c_cc[VMIN]  = 0; // read doesn't block
        tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
        tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
        tty.c_cflag &= ~(PARENB | PARODD); // no parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CRTSCTS; // no hardware flowcontrol

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            ROS_ERROR("Failed to set serial attributes: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // Clear input buffer
        tcflush(serial_fd_, TCIFLUSH);

        ROS_INFO("Successfully opened and configured serial port: %s", port.c_str());
        return true;
    }

    // Convert bytes to int16 (little endian)
    int16_t bytesToInt16LE(const uint8_t* data) {
        return (int16_t)((data[1] << 8) | data[0]);
    }

    // Convert bytes to uint16 (little endian)
    uint16_t bytesToUint16LE(const uint8_t* data) {
        return (uint16_t)((data[1] << 8) | data[0]);
    }

    // 增强的数据验证函数
    bool validateFrame(const uint8_t* frame, size_t length) {
        if (length < FRAME_LENGTH) return false;

        // 验证坐标在合理范围内
        int16_t x = bytesToInt16LE(frame);
        int16_t y = bytesToInt16LE(frame + 2);

        // 假设场地尺寸在±50米内
        if (x < -5000 || x > 5000 || y < -5000 || y > 5000) {
            return false;
        }

        // 验证yaw角度在合理范围内 (0-36000)
        uint16_t yaw = bytesToUint16LE(frame + 4);
        if (yaw > 36000) {
            return false;
        }

        // 验证距离值为正且合理
        for (int i = 0; i < 6; i++) {
            int16_t dist = bytesToInt16LE(frame + 6 + i*2);
            if (dist < 0 || dist > 50000) { // 假设最大距离50米
                return false;
            }
        }

        // 验证错误状态字段
        uint16_t error_status = bytesToUint16LE(frame + 18);

        // 如果bit0=1表示测距失败，数据不可靠
        if (error_status & 0x01) {
            return false;
        }

        // 检查信号强度，如果太差也拒绝
        uint8_t signal_strength = (error_status >> 14) & 0x03;
        if (signal_strength == 0) { // 信号强度差
            return false;
        }

        return true;
    }

    void run() {
        if (!serial_connected_) {
            ROS_ERROR("Serial not connected, cannot run");
            return;
        }

        std::vector<uint8_t> buffer;
        geometry_msgs::Pose2D pose_msg;

        ROS_INFO("Starting UWB tag data parsing...");
        ROS_INFO("Frame length: %d bytes", FRAME_LENGTH);

        int valid_frame_count = 0;
        int total_frame_count = 0;
        ros::Time last_print = ros::Time::now();

        while (ros::ok() && serial_connected_) {
            uint8_t read_buf[256];
            int bytes_read = read(serial_fd_, read_buf, sizeof(read_buf));

            if (bytes_read > 0) {
                // Add to buffer
                buffer.insert(buffer.end(), read_buf, read_buf + bytes_read);

                // 显示原始数据用于调试（仅第一次）
                static bool first_data = true;
                if (first_data && buffer.size() >= 32) {
                    ROS_INFO("First 32 bytes of raw data:");
                    for (size_t i = 0; i < std::min(buffer.size(), size_t(32)); i++) {
                        printf("%02X ", buffer[i]);
                        if ((i+1) % 16 == 0) printf("\n");
                    }
                    printf("\n");
                    first_data = false;
                }

                // 尝试解析帧
                while (buffer.size() >= FRAME_LENGTH) {
                    total_frame_count++;
                    bool frame_found = false;

                    // 滑动窗口寻找有效帧
                    for (size_t start = 0; start <= buffer.size() - FRAME_LENGTH; start++) {
                        if (validateFrame(&buffer[start], buffer.size() - start)) {
                            // 找到有效帧，解析数据
                            if (parseFrame(&buffer[start], buffer.size() - start, pose_msg)) {
                                // 发布位姿
                                pose_pub_.publish(pose_msg);

                                valid_frame_count++;

                                // 每秒输出一次状态
                                ros::Time now = ros::Time::now();
                                if ((now - last_print).toSec() > 1.0) {
                                    float valid_ratio = (float)valid_frame_count / total_frame_count * 100;
                                    ROS_INFO("UWB Pose - x: %.1f cm, y: %.1f cm (valid: %d/%d, %.1f%%)",
                                             pose_msg.x, pose_msg.y,
                                             valid_frame_count, total_frame_count, valid_ratio);
                                    last_print = now;
                                }

                                // 移除已处理的数据
                                buffer.erase(buffer.begin(), buffer.begin() + start + FRAME_LENGTH);
                                frame_found = true;
                                break;
                            }
                        }
                    }

                    if (!frame_found) {
                        // 没找到有效帧，丢弃第一个字节继续尝试
                        if (buffer.size() > 0) {
                            buffer.erase(buffer.begin());
                        }
                    }
                }

                // 防止缓冲区过大
                if (buffer.size() > 1000) {
                    ROS_WARN("Buffer too large (%zu bytes), clearing", buffer.size());
                    buffer.clear();
                }
            } else if (bytes_read < 0) {
                if (errno != EAGAIN) {
                    ROS_ERROR("Serial read error: %s", strerror(errno));
                    break;
                }
            }

            ros::Duration(0.001).sleep();
        }

        ROS_INFO("UWB parser stopped. Valid frames: %d/%d", valid_frame_count, total_frame_count);
    }

    bool parseFrame(const uint8_t* frame, size_t length, geometry_msgs::Pose2D& pose) {
        if (length < FRAME_LENGTH) return false;

        try {
            // 解析坐标x (厘米) - 小端序
            pose.x = bytesToInt16LE(frame);

            // 解析坐标y (厘米) - 小端序
            pose.y = bytesToInt16LE(frame + 2);

            // 不考虑yaw，设置为0
            pose.theta = 0;

            // 解析错误状态和信号强度
            uint16_t error_status = bytesToUint16LE(frame + 18);

            // 输出调试信息（每5秒一次）
            ROS_DEBUG_THROTTLE(5.0, "Parsed frame - x: %d, y: %d, status: 0x%04X",
                               (int)pose.x, (int)pose.y, error_status);

            return true;
        } catch (const std::exception& e) {
            ROS_DEBUG("Frame parsing exception: %s", e.what());
            return false;
        }
    }

    ~UWBTagParser() {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }
};

int main(int argc, char** argv) {
    // Set locale to avoid character encoding issues
    setlocale(LC_ALL, "C");

    ros::init(argc, argv, "uwb_tag_parser");

    UWBTagParser parser;
    parser.run();

    return 0;
}//上面未注释的代码是改进后的，下面的注释掉的有点问题，具体问题：在读取十六进制数据时会处理多余数据，但是是正确的



//#include <ros/ros.h>
//#include <geometry_msgs/Pose2D.h>
//#include <fcntl.h>
//#include <termios.h>
//#include <unistd.h>
//#include <vector>
//#include <iostream>
//#include <string.h>
//#include <stdlib.h>
//
//class UWBTagParser {
//private:
//    ros::NodeHandle nh_;
//    ros::Publisher pose_pub_;
//    int serial_fd_;
//    bool serial_connected_;
//
//    static const int FRAME_LENGTH = 20;
//
//public:
//    UWBTagParser() : serial_fd_(-1), serial_connected_(false) {
//        // 检查ROS Master连接
//        if (!ros::master::check()) {
//            ROS_ERROR("Cannot connect to ROS Master! Please run roscore first.");
//            return;
//        }
//
//        ROS_INFO("Connected to ROS Master");
//
//        // 初始化发布器
//        pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("uwb_pose", 10);
//
//        // 尝试打开串口
//        if (!connectSerial()) {
//            ROS_ERROR("Serial connection failed");
//            return;
//        }
//
//        serial_connected_ = true;
//        ROS_INFO("UWB Tag Parser initialized successfully");
//    }
//
//    bool connectSerial() {
//        std::string port;
//        nh_.param<std::string>("serial_port", port, "/dev/ttyUSB0");
//
//        ROS_INFO("Trying to open serial port: %s", port.c_str());
//
//        // 检查串口设备是否存在
//        if (access(port.c_str(), F_OK) == -1) {
//            ROS_WARN("Serial device not found: %s", port.c_str());
//
//            // 尝试自动检测串口设备
//            std::vector<std::string> possible_ports = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"};
//            bool found = false;
//            for (const auto& p : possible_ports) {
//                if (access(p.c_str(), F_OK) != -1) {
//                    port = p;
//                    ROS_INFO("Found serial device: %s", port.c_str());
//                    found = true;
//                    break;
//                }
//            }
//
//            if (!found) {
//                ROS_ERROR("No serial devices found. Please check connection.");
//                return false;
//            }
//        }
//
//        // 检查权限
//        if (access(port.c_str(), R_OK | W_OK) == -1) {
//            ROS_ERROR("No permission to access %s. Please run:", port.c_str());
//            ROS_ERROR("sudo chmod 666 %s", port.c_str());
//            ROS_ERROR("Or add your user to dialout group: sudo usermod -a -G dialout $USER");
//            return false;
//        }
//
//        // 打开串口
//        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
//        if (serial_fd_ < 0) {
//            ROS_ERROR("Failed to open serial port %s: %s", port.c_str(), strerror(errno));
//            return false;
//        }
//
//        // 配置串口
//        struct termios tty;
//        memset(&tty, 0, sizeof(tty));
//
//        if (tcgetattr(serial_fd_, &tty) != 0) {
//            ROS_ERROR("Failed to get serial attributes: %s", strerror(errno));
//            close(serial_fd_);
//            serial_fd_ = -1;
//            return false;
//        }
//
//        // 设置波特率
//        cfsetospeed(&tty, B115200);
//        cfsetispeed(&tty, B115200);
//
//        // 设置串口参数
//        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 data bits
//        tty.c_iflag &= ~IGNBRK; // dont ignore break
//        tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
//        tty.c_oflag = 0; // no remapping, no delays
//        tty.c_cc[VMIN]  = 0; // read doesn't block
//        tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
//
//        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
//        tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
//        tty.c_cflag &= ~(PARENB | PARODD); // no parity
//        tty.c_cflag &= ~CSTOPB; // 1 stop bit
//        tty.c_cflag &= ~CRTSCTS; // no hardware flowcontrol
//
//        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
//            ROS_ERROR("Failed to set serial attributes: %s", strerror(errno));
//            close(serial_fd_);
//            serial_fd_ = -1;
//            return false;
//        }
//
//        // Clear input buffer
//        tcflush(serial_fd_, TCIFLUSH);
//
//        ROS_INFO("Successfully opened and configured serial port: %s", port.c_str());
//        return true;
//    }
//
//    // Convert bytes to int16 (little endian)
//    int16_t bytesToInt16LE(const uint8_t* data) {
//        return (int16_t)((data[1] << 8) | data[0]);
//    }
//
//    // Convert bytes to uint16 (little endian)
//    uint16_t bytesToUint16LE(const uint8_t* data) {
//        return (uint16_t)((data[1] << 8) | data[0]);
//    }
//
//    // Validate data frame
//    bool validateFrame(const uint8_t* frame, size_t length) {
//        if (length < 16) return false;
//
//        // Validate coordinates are in reasonable range
//        int16_t x = bytesToInt16LE(frame);
//        int16_t y = bytesToInt16LE(frame + 2);
//
//        // Assume field size within ±50 meters
//        if (x < -5000 || x > 5000 || y < -5000 || y > 5000) {
//            return false;
//        }
//
//        return true;
//    }
//
//    void run() {
//        if (!serial_connected_) {
//            ROS_ERROR("Serial not connected, cannot run");
//            return;
//        }
//
//        std::vector<uint8_t> buffer;
//        geometry_msgs::Pose2D pose_msg;
//
//        ROS_INFO("Starting UWB tag data parsing...");
//
//        int frame_count = 0;
//        ros::Time last_print = ros::Time::now();
//
//        while (ros::ok() && serial_connected_) {
//            uint8_t read_buf[256];
//            int bytes_read = read(serial_fd_, read_buf, sizeof(read_buf));
//
//            if (bytes_read > 0) {
//                // Add to buffer
//                buffer.insert(buffer.end(), read_buf, read_buf + bytes_read);
//
//                // Print raw data for debugging (first time only)
//                if (frame_count == 0 && buffer.size() > 0) {
//                    ROS_INFO("First raw data received (%zu bytes):", buffer.size());
//                    for (size_t i = 0; i < std::min(buffer.size(), size_t(32)); i++) {
//                        printf("%02X ", buffer[i]);
//                        if ((i+1) % 16 == 0) printf("\n");
//                    }
//                    printf("\n");
//                }
//
//                // Try to find and parse frames
//                while (buffer.size() >= 16) {
//                    bool frame_found = false;
//
//                    // Slide window to find valid frame
//                    for (size_t start = 0; start <= buffer.size() - 16; start++) {
//                        if (validateFrame(&buffer[start], buffer.size() - start)) {
//                            // Found valid frame, parse it
//                            if (parseFrame(&buffer[start], buffer.size() - start, pose_msg)) {
//                                // Publish pose
//                                pose_pub_.publish(pose_msg);
//
//                                frame_count++;
//                                ros::Time now = ros::Time::now();
//
//                                // Print status every second
//                                if ((now - last_print).toSec() > 1.0) {
//                                    ROS_INFO("UWB Pose - x: %.2f cm, y: %.2f cm (frames: %d)",
//                                             pose_msg.x, pose_msg.y, frame_count);
//                                    last_print = now;
//                                }
//
//                                // Remove processed data
//                                buffer.erase(buffer.begin(), buffer.begin() + start + 16);
//                                frame_found = true;
//                                break;
//                            }
//                        }
//                    }
//
//                    if (!frame_found) {
//                        // No valid frame found, discard first byte
//                        if (buffer.size() > 0) {
//                            buffer.erase(buffer.begin());
//                        }
//                    }
//                }
//
//                // Prevent buffer from growing too large
//                if (buffer.size() > 1000) {
//                    ROS_WARN("Buffer too large (%zu bytes), clearing", buffer.size());
//                    buffer.clear();
//                }
//            } else if (bytes_read < 0) {
//                if (errno != EAGAIN) {
//                    ROS_ERROR("Serial read error: %s", strerror(errno));
//                    break;
//                }
//            }
//
//            ros::Duration(0.001).sleep();
//        }
//
//        ROS_INFO("UWB parser stopped. Total frames processed: %d", frame_count);
//    }
//
//    bool parseFrame(const uint8_t* frame, size_t length, geometry_msgs::Pose2D& pose) {
//        if (length < 16) return false;
//
//        try {
//            // Parse x coordinate (cm) - little endian
//            pose.x = bytesToInt16LE(frame);
//
//            // Parse y coordinate (cm) - little endian
//            pose.y = bytesToInt16LE(frame + 2);
//
//            // yaw not used, set to 0
//            pose.theta = 0;
//
//            // Parse distances to 4 base stations
//            int16_t distances[4];
//            for (int i = 0; i < 4; i++) {
//                distances[i] = bytesToInt16LE(frame + 4 + i*2);
//            }
//
//            // Parse error status
//            uint16_t error_status = bytesToUint16LE(frame + 12);
//
//            ROS_DEBUG_THROTTLE(5.0, "Parsed: x=%d, y=%d, distances: %d,%d,%d,%d, status: 0x%04X",
//                               (int)pose.x, (int)pose.y,
//                               distances[0], distances[1], distances[2], distances[3],
//                               error_status);
//
//            return true;
//        } catch (const std::exception& e) {
//            ROS_ERROR("Frame parsing exception: %s", e.what());
//            return false;
//        }
//    }
//
//    ~UWBTagParser() {
//        if (serial_fd_ >= 0) {
//            close(serial_fd_);
//        }
//    }
//};
//
//int main(int argc, char** argv) {
//    // Set locale to avoid character encoding issues
//    setlocale(LC_ALL, "C");
//
//    ros::init(argc, argv, "uwb_tag_parser");
//
//    UWBTagParser parser;
//    parser.run();
//
//    return 0;
//}
