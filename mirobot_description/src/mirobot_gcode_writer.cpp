#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

serial::Serial _serial;				// serial object

class MirobotWriteNode : public rclcpp::Node
{
public:
    MirobotWriteNode()
        : Node("mirobot_write_node")
    {
        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&MirobotWriteNode::joint_state_callback, this, std::placeholders::_1));
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

        sprintf(angle0, "%.2f", msg->position[0]*57.296);
        sprintf(angle1, "%.2f", msg->position[1]*57.296);
        sprintf(angle2, "%.2f", msg->position[2]*57.296);
        sprintf(angle3, "%.2f", msg->position[3]*57.296);
        sprintf(angle4, "%.2f", msg->position[4]*57.296);
        sprintf(angle5, "%.2f", msg->position[5]*57.296);
        Gcode = (std::string)"M50 G0 X" + angle0 + " Y" + angle1 + " Z" + angle2 + " A" + angle3 + "B" + angle4 + "C" + angle5 + " F3000" + "\r\n";
        
        RCLCPP_INFO(this->get_logger(), "%s", Gcode.c_str());

        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());

        // const auto& joint_names = msg->name;
        // const auto& joint_positions = msg->position;

        // RCLCPP_INFO(this->get_logger(), "Received joint states:");
        // for (size_t i = 0; i < joint_names.size(); ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "%s: %f", joint_names[i].c_str(), joint_positions[i]);
        // }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    std_msgs::msg::String result;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto mirobot_gcode_write_node = std::make_shared<MirobotWriteNode>();

	try{
        //TODO: port name into launch param
		_serial.setPort("/dev/ttyUSB0");
		_serial.setBaudrate(115200);

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

    rclcpp::spin(mirobot_gcode_write_node);
    rclcpp::shutdown();
    
    return 0;
}
