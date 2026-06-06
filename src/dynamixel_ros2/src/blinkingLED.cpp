/**
 * This example is created to demonstrate how to use the library with ROS. 
 * In this case, a ROS node is responsible for switching the integrated LED state every 0.5s,
 * so the user can launch this example to identificate every motors ID without scanning entire system.
 * 
 * 1. Launch roscore:
 *      roscore
 * 
 * 2. Launch the example:
 *      rosrun dynamixel_ros2 blinkingLED  YOUR_PORT PROTOCOL_TYPE BAUDRATE DMXL_ID
*/

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_ros2.h>

#include <cstdlib>   // atoi/atof
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        fprintf(stderr, "Usage: %s PORT_NAME PROTOCOL_VERSION BAUDRATE DMXL_ID\n", argv[0]);
        return 0;
    }

    char* port_name = argv[1];
    float protocol_version = static_cast<float>(std::atof(argv[2]));
    int baud_rate = std::atoi(argv[3]);
    int dmxl_id = std::atoi(argv[4]);

    // create motor as shared pointer so timer callback can access it safely
    auto motor = std::make_shared<dynamixelMotor>("J1", dmxl_id);

    if (!dynamixelMotor::iniComm(port_name, protocol_version, baud_rate)) {
        fprintf(stderr, "Failed to initialize communication on port %s\n", port_name);
        return 1;
    }

    motor->setControlTable();

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("blinkingLED");

    // timer toggles LED every 500 ms
    auto timer = node->create_wall_timer(500ms, [motor, node]() {
        motor->setLedState(!motor->getLedState());
        RCLCPP_INFO(node->get_logger(), "Toggled LED on ID %d", motor->getID());
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
