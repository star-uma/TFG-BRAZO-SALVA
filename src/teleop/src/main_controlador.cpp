#include "manipulador_teleop/controlador.hpp"

int main ( int argc, char * argv[] )
{
    rclcpp::init ( argc, argv );
    
    auto node=std::make_shared<Controlador>();

    rclcpp::Rate rate(50); // 50 Hz

    //node->initialice();  // aquí ya existe el shared_ptr

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        //node->algoritmo_control();
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}