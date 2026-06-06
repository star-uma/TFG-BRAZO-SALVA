/**
 * This example is created to demonstrate how to use the library with ROS. 
 * In this case, a ROS node is responsible for checking permanently a 'user_position_input' topic
 * and to change the DMXLs position if there is some information in the topic.
 * 
 * 1. Launch roscore:
 *      roscore
 * 
 * 2. Launch PlotJuggler (only if you want to visualize some data):
 *      rosrun plotjuggler plotjuggler 
 * 
 * 3. Launch testPositionControl:
 *      rosrun dynamixel_ros2 testPositionControl  YOUR_PORT PROTOCOL_TYPE BAUDRATE DMXL_ID
 * 
 * 4. Publish any position between 0 and 360 degrees in the created topic:
 *      rostopic pub -1 /user_position_input std_msgs/Int32 "data: DEGREES"
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <dynamixel_ros2.h>
#include <cstdlib> // atoi/atof

dynamixelMotor myDynamixel;

void userInputCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int userValue = msg->data;

    if(myDynamixel.getOperatingMode() != "Position Control")
    {
        if(myDynamixel.getTorqueState())
        {
            myDynamixel.setTorqueState(false);
        }

        myDynamixel.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    }

    myDynamixel.setTorqueState(true);
    myDynamixel.setGoalPosition(userValue);
}

int main(int argc, char **argv)
{
    char* port_name;
    int baud_rate, dmxl_id;
    float protocol_version;

    if (argc != 5)
    {
        printf("Please set '-port_name', '-protocol_version' '-baud_rate' '-dynamixel_id' arguments for connected Dynamixels\n");
        return 0;
    } else
    {
        port_name = argv[1];
        protocol_version = static_cast<float>(std::atof(argv[2]));
        baud_rate = std::atoi(argv[3]);
        dmxl_id = std::atoi(argv[4]);
    }

    myDynamixel = dynamixelMotor("example",dmxl_id);
    dynamixelMotor::iniComm(port_name,protocol_version,baud_rate);
    myDynamixel.setControlTable();

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("goal_position_reader");
    auto sub = node->create_subscription<std_msgs::msg::Int32>(
      "user_position_input", 10, userInputCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
