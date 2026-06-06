#pragma once

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>


#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


// KDL

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>


#include <string>
#include <vector>
#include <optional>
#include <cmath>

class Controlador : public rclcpp::Node
{
public:
    Controlador();
    ~Controlador();

    // ── SUBSCRIPTORES ──────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // =================== PUBLICADORES =============================

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;

    // ── TIMER ──────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr timer_;
    void timerCallback();

private:
    
    bool initKDL();
    void publishZeroVelocities();

    // ================================= KDL ===========================
    KDL::Chain   chain_;
    KDL::JntArray q_current_;      // posiciones actuales (rad)
    std::shared_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;   //ChainIkSolverVel_pinv
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

    KDL::Twist twist;   // inicializado a cero

    



    // ===========================PARÁMETROS DE CONTROL ================

    double linear_velocity;   // m/s
    double angular_velocity;  // rad/s
    double joint_velocity;    // rad/s
    double deadzone;
    double control_rate_hz;

    // ===============================ESTADO ===========================

    bool recibido;
    bool modo_cartesiano;    
    bool modo_orientacion;    
    int  modo_operacion;
    int tiempo_boton;
    
   
    std::optional<sensor_msgs::msg::Joy> joy_msg_;
    
    int garra;






    // Nombres de joints en el mismo orden que el SRDF
    std::vector<std::string> joint_names_;
    int n_joints_;
};