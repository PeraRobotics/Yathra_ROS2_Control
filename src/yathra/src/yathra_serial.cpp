#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
// using Float32MultiArray for generic commands
#include <std_msgs/msg/float32_multi_array.hpp> 

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

// --- Protocol Definitions ---
#define RX_HEADER_TAG "SUB_TEL" // 7 chars + null terminator = 8 bytes
#define TX_HEADER_TAG "SUB_CMD"

// Must match ESP32 struct exactly
#pragma pack(push, 1)
struct ImuDataPacket {
    char tag[8];
    float accel[3]; // x, y, z
    float gyro[3];  // x, y, z
    float mag[3];   // x, y, z
    float roll;
    float pitch;
    float head;
    float temp;
};

struct CommandDataPacket {
    char tag[8];
    // TODO: Define your actual command structure here. 
    // Assuming 4 floats for example (e.g. motor speeds)
    float values[4]; 
};
#pragma pack(pop)

class YathraSerialNode : public rclcpp::Node {
public:
    YathraSerialNode() : Node("yathra_serial") {
        // --- Parameters ---
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);

        std::string port = this->get_parameter("port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        // --- Serial Port Init ---
        if (!open_serial_port(port, baud_rate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            // In a real robot, usually we'd exit or retry, but we'll let the node spin
        } else {
            RCLCPP_INFO(this->get_logger(), "Connected to %s at %d", port.c_str(), baud_rate);
        }

        // --- Publishers ---
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/yathra/raw_imu", 10);
        attitude_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/yathra/attitude", 10);
        temp_pub_ = this->create_publisher<std_msgs::msg::Float32>("/yathra/tube_conditions", 10);
        log_pub_ = this->create_publisher<std_msgs::msg::String>("/yathra/logs", 10);

        // --- Subscriber ---
        // Expecting an array of floats to send as command
        cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/yathra/cmd", 10,
            std::bind(&YathraSerialNode::command_callback, this, std::placeholders::_1)
        );

        // --- Reader Thread ---
        // We run the read loop in a separate thread to not block ROS executors
        running_ = true;
        read_thread_ = std::thread(&YathraSerialNode::read_loop, this);
    }

    ~YathraSerialNode() {
        running_ = false;
        if (read_thread_.joinable()) read_thread_.join();
        if (serial_fd_ >= 0) close(serial_fd_);
    }

private:
    int serial_fd_ = -1;
    std::thread read_thread_;
    std::atomic<bool> running_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;

    bool open_serial_port(const std::string& port, int baud_rate) {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) return false;

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;

        cfsetospeed(&tty, (speed_t)B115200); // Standard Linux defines, usually switch-case needed for variable baud
        cfsetispeed(&tty, (speed_t)B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
        tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
        tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CRTSCTS; // no flow control

        // Raw input mode
        tty.c_lflag = 0; 
        tty.c_oflag = 0;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        // Non-blocking read with timeout logic handled manually usually, 
        // but here we set VMIN=0, VTIME=1 (0.1s timeout)
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1; 

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;
        return true;
    }

    void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (serial_fd_ < 0) return;

        CommandDataPacket tx_packet;
        memset(&tx_packet, 0, sizeof(tx_packet));
        
        // 1. Set Header
        strncpy(tx_packet.tag, TX_HEADER_TAG, 8);

        // 2. Fill Data (Safety check size)
        size_t count = std::min((size_t)4, msg->data.size()); // Assuming 4 floats in struct
        for(size_t i=0; i<count; i++) {
            tx_packet.values[i] = msg->data[i];
        }

        // 3. Send
        write(serial_fd_, &tx_packet, sizeof(tx_packet));
    }

    void read_loop() {
        std::vector<uint8_t> buffer;
        const size_t packet_size = sizeof(ImuDataPacket);
        uint8_t temp_buf[256];

        while (running_ && rclcpp::ok()) {
            if (serial_fd_ < 0) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            // Read bytes
            int n = read(serial_fd_, temp_buf, sizeof(temp_buf));
            if (n > 0) {
                buffer.insert(buffer.end(), temp_buf, temp_buf + n);
            }

            // Parse Buffer
            while (buffer.size() >= packet_size) {
                // Check Header "SUB_TEL"
                bool header_match = true;
                const char* header = RX_HEADER_TAG;
                
                // Compare first 7 bytes (ignore null terminator for flexible matching if needed, but strict is better)
                if (memcmp(buffer.data(), header, 7) == 0) {
                    
                    // We found a packet!
                    ImuDataPacket* pkt = reinterpret_cast<ImuDataPacket*>(buffer.data());

                    publish_data(pkt);

                    // Remove this packet from buffer
                    buffer.erase(buffer.begin(), buffer.begin() + packet_size);

                } else {
                    // Invalid Header logic
                    // If we have enough bytes for a header but it doesn't match, 
                    // check if it MIGHT be another type of log, or just noise.
                    
                    // Simple logic: If it's definitely not SUB_TEL, maybe log it?
                    // But logging every byte shift is noisy. 
                    // We log only if we find a "wrong" header that looks like text.
                    
                    // Shift buffer by 1 byte to keep searching
                    buffer.erase(buffer.begin());
                }
            }
            
            // Avoid CPU hogging
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void publish_data(ImuDataPacket* pkt) {
        auto current_time = this->now();

        // 1. Publish Raw IMU
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = current_time;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.linear_acceleration.x = pkt->accel[0];
        imu_msg.linear_acceleration.y = pkt->accel[1];
        imu_msg.linear_acceleration.z = pkt->accel[2];
        imu_msg.angular_velocity.x = pkt->gyro[0];
        imu_msg.angular_velocity.y = pkt->gyro[1];
        imu_msg.angular_velocity.z = pkt->gyro[2];
        imu_pub_->publish(imu_msg);

        // 2. Publish Attitude (Euler)
        geometry_msgs::msg::Vector3 att_msg;
        att_msg.x = pkt->roll;
        att_msg.y = pkt->pitch;
        att_msg.z = pkt->head;
        attitude_pub_->publish(att_msg);

        // 3. Publish Temp
        std_msgs::msg::Float32 temp_msg;
        temp_msg.data = pkt->temp;
        temp_pub_->publish(temp_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YathraSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}