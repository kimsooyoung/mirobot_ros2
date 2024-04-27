#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

serial::Serial _serial;				// serial object

double truncateDecimal(double value, int precision) {
    double factor = pow(10.0, precision);
    return floor(value * factor) / factor;
}

class MirobotWriteNode : public rclcpp::Node
{
public:
    MirobotWriteNode()
        : Node("mirobot_write_node")
    {
        this->declare_parameter("joint_states_topic_name", "/issac/joint_states");
        // this->declare_parameter("joint_states_topic_name", "/issac/joint_command");
        
        joint_states_topic_name = this->get_parameter("joint_states_topic_name").as_string();
        RCLCPP_INFO(this->get_logger(), "Joint States Topic Name : %s", joint_states_topic_name.c_str());

        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_name, 10, std::bind(&MirobotWriteNode::joint_state_callback, this, std::placeholders::_1));
    }

    void homing(){
        //TODO: home position reset
        // https://document.wlkata.com/?doc=/wlkata-mirobot-user-manual-platinum/18-g-code-instruction-set/
        std::string HomingGcode = (std::string)"$H" + "\r\n";

        _serial.write(HomingGcode.c_str());
        result.data = _serial.read(_serial.available());

        RCLCPP_INFO(this->get_logger(), "%s", result.data.c_str());
        RCLCPP_INFO(this->get_logger(), "Wait for seconds, Mirobot is Homing now...");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::string Gcode = "";
        char angle0[10];
        char angle1[10];
        char angle2[10];
        char angle3[10];
        char angle4[10];
        char angle5[10];

        auto position0 = truncateDecimal(msg->position[0]*57.296, 2);
        auto position1 = truncateDecimal(msg->position[1]*57.296, 2);
        auto position2 = truncateDecimal(msg->position[2]*57.296, 2);
        auto position3 = truncateDecimal(msg->position[3]*57.296, 2);
        auto position4 = truncateDecimal(msg->position[4]*57.296, 2);
        auto position5 = truncateDecimal(msg->position[5]*57.296, 2);

        sprintf(angle0, "%.2f", position0);
        sprintf(angle1, "%.2f", position1);
        sprintf(angle2, "%.2f", position2);
        sprintf(angle3, "%.2f", position3);
        sprintf(angle4, "%.2f", position4);
        sprintf(angle5, "%.2f", position5);
        Gcode = (std::string)"M50 G0 X" + angle0 + " Y" + angle1 + " Z" + angle2 + " A" + angle3 + "B" + angle4 + "C" + angle5 + " F3000" + "\r\n";
        
        RCLCPP_INFO(this->get_logger(), "%s", Gcode.c_str());
        RCLCPP_INFO(this->get_logger(), "position0 : %f / position1 : %f / position2 : %f", position0, position1, position2);

        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    std::string joint_states_topic_name;
    std_msgs::msg::String result;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto mirobot_gcode_write_node = std::make_shared<MirobotWriteNode>();
    
    mirobot_gcode_write_node->declare_parameter("port_name", "/dev/ttyUSB0");
    mirobot_gcode_write_node->declare_parameter("baud_rate", 115200);

    auto port_name = mirobot_gcode_write_node->get_parameter("port_name").as_string();
    auto baud_rate = mirobot_gcode_write_node->get_parameter("baud_rate").as_int();

    RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Port Name : %s", port_name.c_str());
    RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Baudrate : %d", baud_rate);

	try{
		_serial.setPort(port_name);
		_serial.setBaudrate(baud_rate);

		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		
        _serial.setTimeout(to);
		_serial.open();
		_serial.write("M50\r\n");
		
        RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Port has been open successfully");
	}
	catch (serial::IOException& e){
        RCLCPP_FATAL(mirobot_gcode_write_node->get_logger(), "Unable to open port");
		return -1;
	}

	if (_serial.isOpen()){
        using namespace std::chrono_literals;
        rclcpp::sleep_for(1s);
        RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Attach and wait for commands");
	}

    mirobot_gcode_write_node->homing();
    rclcpp::sleep_for(std::chrono::seconds(13));
    RCLCPP_WARN(mirobot_gcode_write_node->get_logger(), "Homing Done!!!");

    rclcpp::spin(mirobot_gcode_write_node);
    rclcpp::shutdown();
    
    return 0;
}
