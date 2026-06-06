#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"

#include "api.h"   // API de Rozum Robotics

#include <dynamixel_ros2.h>
#include <cstdlib> 
#include <iostream>

#include <stdlib.h>
#include <vector>
#include <string>
#include <cmath>
#include <memory>

//Joystick
#include <sensor_msgs/msg/joy.hpp> 

namespace hardware{

class HwInterface : public hardware_interface::SystemInterface{

public:

    RCLCPP_SHARED_PTR_DEFINITIONS(HwInterface)

    ~HwInterface();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/) override;

private:


    inline double deg2rad(double deg);

    inline double rad2deg(double deg);

    inline double rad2rpm(double rad);
    
    inline void updateDxlCache();

    void torqueEnabled();
    
    void torqueDisabled();


    //==============HW_INTERFACE===============

   
   

    //===================BUS CAN==============

    // 1 servo
    rr_can_interface_t *iface;
    rr_servo_t *servo[3];

    // IDs
    uint8_t ids_[3];

    // Interfaz CAN
    std::string interfazCan;
    
    //=================DINAMIXEL==============

    dynamixelMotor motorJ0;
    dynamixelMotor motorJ1;
    dynamixelMotor motorJ2;
    dynamixelMotor motorJ3;
    dynamixelMotor motorJ4;

    //================OTROS===================


    // Estados
    double pos_[8];
    double vel_[8];
    // double eff_[6];

    // Comandos
    double cmd_vel_[8];

    double target_vel_deg[8];
};



}   // namespace hardware