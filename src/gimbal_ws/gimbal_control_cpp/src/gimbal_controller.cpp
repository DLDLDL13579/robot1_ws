#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

class GimbalController : public rclcpp::Node{
public:
    GimbalController() : Node("gimbalcontroller"),serial_fd_(-1){   //This
        offset_yaw_ = 0.0;
        offset_pitch_ = 0.0;
        yaw_level_deg_ = this->declare_parameter<double>("yaw_level_deg", 0.0);
        pitch_level_deg_ = this->declare_parameter<double>("pitch_level_deg", 0.0);
        move_to_level_on_start_ = this->declare_parameter<bool>(
    "move_to_level_on_start", false
);
has_command_ = move_to_level_on_start_;

if (move_to_level_on_start_) {
    RCLCPP_INFO(this->get_logger(), "Moving to calibrated level on startup.");
}
    const std::string port = this->declare_parameter<std::string>(
        "port_name",
        "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B21242534-if00");
    const int baudrate = this->declare_parameter<int>("baud_rate", 115200);
        RCLCPP_INFO(this->get_logger(),"Opening serial port: %s...",port.c_str());
        serial_fd_ = open_serial(port,baudrate);
        if(serial_fd_ < 0){
            RCLCPP_ERROR(this->get_logger(),"Faild to open serial! Error: %d",errno);
        }
        init_gimbal();
        manual_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "cmd_offset", //This
            10,
            std::bind(&GimbalController::manual_offset_callback,this,std::placeholders::_1)//This
        );
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GimbalController::control_loop,this)
        );
        RCLCPP_INFO(this->get_logger(),"System runing. Ready for offset commands.");
    }
    ~GimbalController(){
        if(serial_fd_ >= 0 ){
            RCLCPP_INFO(this->get_logger(),"Disabling motors before exit...");
            send_command(0x01,0x05);
            send_command(0x02,0x05);
            close(serial_fd_);
        }
    }
private:
    int serial_fd_;
    double offset_yaw_; //This
    double offset_pitch_;
    double yaw_level_deg_;
    double pitch_level_deg_;
    bool move_to_level_on_start_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr manual_sub_;   //This
    rclcpp::TimerBase::SharedPtr timer_;
    bool has_command_{false};
    int open_serial(const std::string& port_name,int baudrate){
        int fd = open(port_name.c_str(),O_RDWR|O_NOCTTY);//This
        if(fd == -1)return -1;
        struct termios options;    //THis
        tcgetattr(fd,&options);     //This
        speed_t speed = B115200;   //This
        cfsetispeed(&options,speed);
        cfsetospeed(&options,speed);
        options.c_cflag |= (CLOCAL | CREAD); // 保证程序不会占用串口，且可以读取数据
        options.c_cflag &= ~PARENB;          // 无奇偶校验位
        options.c_cflag &= ~CSTOPB;          // 1个停止位
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;              // 8个数据位
        // 【关键修复】：彻底禁用软硬件流控制，保证二进制数据不会被过滤或拦截
        options.c_cflag &= ~CRTSCTS;                                 // 禁用硬件流控
        options.c_iflag &= ~(IXON | IXOFF | IXANY);                  // 禁用软件流控
        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // 原始输入模式
        
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);          // 选择原始输入（不以回车作为一行）
        options.c_oflag &= ~OPOST;                                   // 选择原始输出
        tcsetattr(fd, TCSANOW, &options);
        tcflush(fd, TCIOFLUSH); // 清空输入输出缓冲区
        return fd;
    }
    uint8_t calculate_bcc(const std::vector<uint8_t>&packet){
        uint8_t bcc = 0;
        for (uint8_t b : packet) {
            bcc ^= b;
        }
        return bcc;
    }
    void send_command(uint8_t addr,uint8_t cmd_type,const std::vector<uint8_t>& data = {}){
        if(serial_fd_ < 0 )return;
        std::vector<uint8_t>packet = {0x7A,addr,cmd_type};
        if(!data.empty()){
            packet.insert(packet.end(),data.begin(),data.end());
        }
        uint8_t bcc = calculate_bcc(packet);
        packet.push_back(bcc);
        packet.push_back(0x7B);
        write(serial_fd_,packet.data(),packet.size());
        tcdrain(serial_fd_);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    void send_angle_target(uint8_t addr, double angle_deg) {
        // 数据需要放大 10 倍
        int32_t val_int = static_cast<int32_t>(std::round(angle_deg * 10.0));
        
        std::vector<uint8_t> data(4);
        data[0] = (val_int >> 24) & 0xFF;
        data[1] = (val_int >> 16) & 0xFF;
        data[2] = (val_int >> 8) & 0xFF;
        data[3] = val_int & 0xFF;
        
        send_command(addr, 0x02, data); // 功能码 0x02 代表设置多圈绝对角度
    }
    void init_gimbal(){
        RCLCPP_INFO(this->get_logger(),"Enableing motors...");
        send_command(0x01,0x06);
        send_command(0x02,0x06);
        RCLCPP_INFO(this->get_logger(),"Configuring motoin curve...");
        send_command(0x01,0x00,{0x00,0x01});
        send_command(0x02,0x00,{0x00,0x01});
         // ==================== 【关键新增步骤】 ====================
        RCLCPP_INFO(this->get_logger(), "Setting speed limit to 100 RPM...");
        // 目标速度功能码为 0x01，数据为 2 字节（大端格式）
        // 100 RPM 的十六进制为 0x0064，拆分为数据字节 {0x00, 0x64}
        send_command(0x01, 0x01, {0x00, 0x64}); // 给 X轴 设置最大速度为 100 RPM
        send_command(0x02, 0x01, {0x00, 0x64}); // 给 Y轴 设置最大速度为 100 RPM
        // ========================================================
    }
    void manual_offset_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
        offset_yaw_ = msg->x;
        offset_pitch_ = msg->y;
        has_command_ = true;
        RCLCPP_INFO(this->get_logger(),"Recived offset: Yaw = %.1f Pitch = %.1f",offset_yaw_,offset_pitch_);
    }
    void control_loop(){
        if (!has_command_) {
            return;
            }
        send_angle_target(1, yaw_level_deg_ + offset_yaw_);
        send_angle_target(2, pitch_level_deg_ + offset_pitch_);
    }
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<GimbalController>());
    rclcpp::shutdown();
    return 0;
}

//你好