#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <tf/transform_datatypes.h>  // 加入用于四元数转欧拉角
 
struct DataBuffer {
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = 0.0;
    double yaw = 0.0;
    bool odom_received = false;
    bool cmd_received = false;
};
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_sender_minimal");
    ros::NodeHandle nh;
 
    serial::Serial serial_port;
    try {
        serial_port.setPort("/dev/ttyUSB0");
        serial_port.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(timeout);
        serial_port.open();
    }
    catch (serial::IOException& e) {
        ROS_ERROR_STREAM("无法打开串口: " << e.what());
        return 1;
    }
 
    DataBuffer buffer;
 
    // 订阅 odometry
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 10,
        [&](const nav_msgs::Odometry::ConstPtr& msg) {
            buffer.x = msg->pose.pose.position.x;
            buffer.y = msg->pose.pose.position.y;
 
            // 提取四元数并转换为偏航角
            tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            buffer.yaw = yaw;
 
            buffer.odom_received = true;
 
            if (buffer.cmd_received) {
                std::string data = "x:" + std::to_string(buffer.x) +
                    ",y:" + std::to_string(buffer.y) +
                    ",yaw:" + std::to_string(buffer.yaw) +
                    ",vx:" + std::to_string(buffer.vx) +
                    ",vy:" + std::to_string(buffer.vy) +
                    ",yaw_rate:" + std::to_string(buffer.yaw_rate) + "\n";
                serial_port.write(data);
                ROS_INFO_STREAM("[Serial Send] " << data);
            }
        });
 
    // 订阅 cmd_vel
    ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
        [&](const geometry_msgs::Twist::ConstPtr& msg) {
            buffer.vx = msg->linear.x;
            buffer.vy = msg->linear.y;
            buffer.yaw_rate = msg->angular.z;
            buffer.cmd_received = true;
        });
 
    ros::spin();
    serial_port.close();
    return 0;
}