#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <dynamixel_ros2.h>
#include <cstdlib>
#include <iostream>

dynamixelMotor motorJ0,motorJ1,motorJ2,motorJ10,motorJ11,motorJ12;

// Callback when some data was published in 'pos_user_input'
// minimal change: use ROS2 message shared pointer type
void callBack(const std_msgs::msg::Int32::SharedPtr msg)
{
    int userValue = msg->data;
    
    motorJ0.setTorqueState(true);
    motorJ10.setTorqueState(true);
    motorJ1.setTorqueState(true);
    motorJ11.setTorqueState(true);
    motorJ2.setTorqueState(true);
    motorJ12.setTorqueState(true);

    switch (userValue)
    {
        case 0:
            motorJ0.setGoalPosition(60);
            motorJ10.setGoalPosition(345);
            motorJ1.setGoalPosition(12);
            motorJ11.setGoalPosition(358);
            motorJ2.setGoalPosition(120);
            motorJ12.setGoalPosition(240);
        break;

        case 1:
            motorJ0.setGoalPosition(180);
            motorJ10.setGoalPosition(240);
            motorJ1.setGoalPosition(120);
            motorJ11.setGoalPosition(280);
            motorJ2.setGoalPosition(240);
            motorJ12.setGoalPosition(120);
        break;

        case 2:
            motorJ0.setGoalPosition(180);
            motorJ10.setGoalPosition(180);
            motorJ1.setGoalPosition(120);
            motorJ11.setGoalPosition(315);
            motorJ2.setGoalPosition(240);
            motorJ12.setGoalPosition(200);
        break;
    
        default:

        break;
    }
}

int main(int argc, char *argv[])
{
    char* port_name;
    int baud_rate;
    float protocol_version;

    if (argc != 4)
    {
        printf("Please set 'PORT_NAME PROTOCOL_VERSION BAUDRATE' arguments for connected Dynamixels\n");
        return 0;
    } else
    {
        port_name = argv[1];
        protocol_version = static_cast<float>(std::atof(argv[2]));
        baud_rate = std::atoi(argv[3]);
    }

    // init comm and motor objects (unchanged)
    dynamixelMotor::iniComm(port_name,protocol_version,baud_rate);
    motorJ0 = dynamixelMotor("J0",0);
    motorJ10 = dynamixelMotor("J10",10);
    motorJ1 = dynamixelMotor("J1",1);
    motorJ11 = dynamixelMotor("J11",11);
    motorJ2 = dynamixelMotor("J2",2);
    motorJ12 = dynamixelMotor("J12",12);

    motorJ0.setControlTable();
    motorJ10.setControlTable();
    motorJ1.setControlTable();
    motorJ11.setControlTable();
    motorJ2.setControlTable();
    motorJ12.setControlTable();
    
    // rclcpp init and node creation (minimal change)
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("testFingers");

    // create subscription using the free-function callback
    auto user_input_subscriber = node->create_subscription<std_msgs::msg::Int32>(
        "pos_user_input", 10, callBack);

    // loop at 10 Hz
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
