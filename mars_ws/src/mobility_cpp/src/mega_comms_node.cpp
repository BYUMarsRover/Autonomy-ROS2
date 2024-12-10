#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <rover_msgs/msg/iwc_motors.hpp>
#include <rover_msgs/msg/elevator.hpp>
#include <serial/serial.h>

class NmeaSerialNode : public rclcpp::Node
{
public:
    NmeaSerialNode() : Node("nmea_serial_node")
    {
        // Set up serial connection
        try {
            ser_.setPort("/dev/mars/mega");
            ser_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ser_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port");
            throw std::runtime_error("Failed to open serial port");
        }

        if(ser_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
        } else {
            throw std::runtime_error("Failed to open serial port");
        }

        // Subscribers
        sub_wheels_ = this->create_subscription<custom_interfaces::msg::IWCmotors>(
            "IWC_motorControl", 10, std::bind(&NmeaSerialNode::wheelCb, this, std::placeholders::_1));
        sub_elev_ = this->create_subscription<custom_interfaces::msg::Elevator>(
            "elevator", 10, std::bind(&NmeaSerialNode::elevatorCb, this, std::placeholders::_1));
        sub_click_ = this->create_subscription<std_msgs::msg::Bool>(
            "arm_clicker", 10, std::bind(&NmeaSerialNode::clickCb, this, std::placeholders::_1));
        sub_laser_ = this->create_subscription<std_msgs::msg::Bool>(
            "arm_laser", 10, std::bind(&NmeaSerialNode::laserCb, this, std::placeholders::_1));

        // Publisher for IR data
        ir_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("IR", 10);

        // Timer for publishing IR data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&NmeaSerialNode::timerCallback, this));
    }

private:
    void wheelCb(const custom_interfaces::msg::IWCmotors::SharedPtr msg)
    {
        std::string sentence = "$WHEEL,";
        sentence += std::to_string(msg->right_front_speed) + "," + std::to_string(msg->right_front_dir) + ",";
        sentence += std::to_string(msg->right_middle_speed) + "," + std::to_string(msg->right_middle_dir) + ",";
        sentence += std::to_string(msg->right_rear_speed) + "," + std::to_string(msg->right_rear_dir) + ",";
        sentence += std::to_string(msg->left_front_speed) + "," + std::to_string(msg->left_front_dir) + ",";
        sentence += std::to_string(msg->left_middle_speed) + "," + std::to_string(msg->left_middle_dir) + ",";
        sentence += std::to_string(msg->left_rear_speed) + "," + std::to_string(msg->left_rear_dir) + "*";
        
        ser_.write(sentence);
    }

    void elevatorCb(const custom_interfaces::msg::Elevator::SharedPtr msg)
    {
        std::string sentence = "$ELEVA," + std::to_string(msg->elevator_speed) + "," + std::to_string(msg->elevator_direction) + "*";
        ser_.write(sentence);
    }

    void clickCb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::string sentence = "$CLICK," + std::to_string(msg->data) + "*";
        ser_.write(sentence);
    }

    void laserCb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::string sentence = "$LASER," + std::to_string(msg->data) + "*";
        ser_.write(sentence);
    }

    void timerCallback()
    {
        auto message = std_msgs::msg::UInt16MultiArray();
        // Here you can read the sensor data and fill the message
        // For example, message.data = ... (Fill with sensor data)

        ir_pub_->publish(message);
    }

    serial::Serial ser_;
    rclcpp::Subscription<custom_interfaces::msg::IWCmotors>::SharedPtr sub_wheels_;
    rclcpp::Subscription<custom_interfaces::msg::Elevator>::SharedPtr sub_elev_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_click_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_laser_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr ir_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NmeaSerialNode>());
    rclcpp::shutdown();
    return 0;
}
