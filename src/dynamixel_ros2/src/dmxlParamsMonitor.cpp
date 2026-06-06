/**
 * This example is created to demonstrate how to use the library with ROS. 
 * In this case, a ROS node is responsible for publishing data on temperature, velocity, 
 * current, and position every 0.1 seconds. Additionally, it listens to a ROS topic named 
 * "pos_user_input," where the user must input the desired motor position. 
 * When a new value appears on this topic, the program executes the corresponding 
 * callback and then resumes its normal routine.
 * 
 * 1. Launch roscore:
 *      roscore
 * 
 * 2. Launch PlotJuggler (only if you want to visualize some data):
 *      rosrun plotjuggler plotjuggler
 *  
 * 3. Launch dmxlParamsMonitor:
 *      rosrun dynamixel_ros2 dmxlParamsMonitor  YOUR_PORT PROTOCOL_TYPE BAUDRATE DMXL_ID
 * 
 * 4. Publish any position between 0 and 360 degrees in the created topic:
 *      rostopic pub -1 /pos_user_input std_msgs/Int32 "data: DEGREES"
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <dynamixel_ros2.h>
#include <memory>
#include <chrono>
#include <cstdlib>
#include <iostream>

using namespace std::chrono_literals;

static rclcpp::Logger logger = rclcpp::get_logger("dmxlParamsMonitor");

// keep the same global motor object
dynamixelMotor myMotor;

// keep similar function signature (use rclcpp publisher shared_ptrs)
void publishMotorStatus(
  const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & pos_pub,
  const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & vel_pub,
  const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & curr_pub,
  const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & temp_pub)
{
  std_msgs::msg::Float32 pos_msg;
  std_msgs::msg::Float32 vel_msg;
  std_msgs::msg::Float32 curr_msg;
  std_msgs::msg::Float32 temp_msg;

  pos_msg.data  = static_cast<float>(myMotor.getPresentPosition());
  vel_msg.data  = static_cast<float>(myMotor.getPresentVelocity());
  curr_msg.data = static_cast<float>(myMotor.getPresentCurrent());
  temp_msg.data = static_cast<float>(myMotor.getPresentTemperature());

  pos_pub->publish(pos_msg);
  vel_pub->publish(vel_msg);
  curr_pub->publish(curr_msg);
  temp_pub->publish(temp_msg);
}

// Callback when some data was published in 'pos_user_input'
void callBack(const std_msgs::msg::Int32::SharedPtr msg)
{
  int userValue = msg->data;
  if (userValue >= 0 && userValue <= 360)
  {
    if (myMotor.getTorqueState())
    {
      myMotor.setTorqueState(false);
    }

    if (myMotor.getOperatingMode() != "EXTENDED_POSITION_CONTROL_MODE")
    {
      myMotor.setOperatingMode(dynamixelMotor::EXTENDED_POSITION_CONTROL_MODE);
    }

    myMotor.setTorqueState(true);
    myMotor.setGoalPosition(userValue);
    RCLCPP_INFO(logger, "Set goal position to %d", userValue);
  }
  else
  {
    RCLCPP_WARN(logger, "Invalid input: %d. Please enter a value between 0 and 360.", userValue);
  }
}

int main(int argc, char *argv[])
{
  if (argc != 5)
  {
    printf("Please set 'PORT_NAME PROTOCOL_VERSION BAUDRATE DMXL_ID' arguments for connected Dynamixels\n");
    return 0;
  }

  char * port_name = argv[1];
  float protocol_version = static_cast<float>(std::atof(argv[2]));
  int baud_rate = std::atoi(argv[3]);
  int dmxl_id = std::atoi(argv[4]);

  // initialize motor object
  myMotor = dynamixelMotor("M1", dmxl_id);

  if (!dynamixelMotor::iniComm(port_name, protocol_version, baud_rate))
  {
    fprintf(stderr, "Failed to initialize communication on port %s\n", port_name);
    return 1;
  }

  myMotor.setControlTable();

  // rclcpp init and node creation
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dmxlParamsMonitor");

  // publishers and subscriber (keep topic names as before)
  auto pos_publisher  = node->create_publisher<std_msgs::msg::Float32>("motor_position", 10);
  auto vel_publisher  = node->create_publisher<std_msgs::msg::Float32>("motor_velocity", 10);
  auto curr_publisher = node->create_publisher<std_msgs::msg::Float32>("motor_current", 10);
  auto temp_publisher = node->create_publisher<std_msgs::msg::Float32>("motor_temperature", 10);

  auto user_input_subscriber = node->create_subscription<std_msgs::msg::Int32>(
    "pos_user_input", 10, [](const std_msgs::msg::Int32::SharedPtr msg){ callBack(msg); });

  // loop at 10 Hz similar to ros::Rate loop
  rclcpp::Rate loop_rate(50);

  RCLCPP_INFO(logger, "dmxlParamsMonitor started for ID %d", dmxl_id);

  while (rclcpp::ok())
  {
    publishMotorStatus(pos_publisher, vel_publisher, curr_publisher, temp_publisher);

    // process incoming messages
    rclcpp::spin_some(node);

    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
