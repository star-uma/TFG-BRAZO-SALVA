#include "hardware/hw_interface.hpp"

namespace hardware{

HwInterface::~HwInterface(){


    //Apagar servos
    //for (int i = 0; i < 3; i++){

        //servo[i] = rr_deinit_servo(iface, ids_[i]);
        

    // Apagar interfaz CAN
    //iface = rr_deinit_interface(interfazCan.c_str());
    
    torqueDisabled();

}


inline double HwInterface::deg2rad(double deg){

    return deg * M_PI / 180.0;

}

inline double HwInterface::rad2deg(double deg){

    return deg * 180 / M_PI ;

}

inline double HwInterface::rad2rpm(double rad){

    return rad * 60 / ( 2 * M_PI ) ;

}

void HwInterface::torqueEnabled(){

    motorJ0.setTorqueState(true);
    motorJ1.setTorqueState(true);
    motorJ2.setTorqueState(true);
    motorJ3.setTorqueState(true);
    motorJ4.setTorqueState(true);
 
}

void HwInterface::torqueDisabled(){

    motorJ0.setTorqueState(false);
    motorJ1.setTorqueState(false);
    motorJ2.setTorqueState(false);
    motorJ3.setTorqueState(false);
    motorJ4.setTorqueState(false);

}


hardware_interface::CallbackReturn HwInterface::on_init(const hardware_interface::HardwareInfo & info){

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){

        return hardware_interface::CallbackReturn::ERROR;
    }

    
    

    //==========================BUS CAN===============================

    interfazCan = "/dev/ttyACM0";

    // IDs de tus 6 servos
    ids_[0] = 123;
    ids_[1] = 124;
    ids_[2] = 125;
   

    // Inicializar arrays
    for (int i = 0; i < 8; i++)
    {
        pos_[i] = 0.0;
        vel_[i] = 0.0;
        cmd_vel_[i] = 0.0;
        
    }


    // Inicializar interfaz CAN
    iface = rr_init_interface(interfazCan.c_str());
    if (!iface){
        
        RCLCPP_ERROR(rclcpp::get_logger("HwInterface"), "Error al iniciar interfaz CAN");
        return hardware_interface::CallbackReturn::ERROR;
    
    }

    // Inicializar servos
    for (int i = 0; i < 3; i++){

        servo[i] = rr_init_servo(iface, ids_[i]);
        if (!servo[i]){
            
            RCLCPP_ERROR(rclcpp::get_logger("HwInterface"), "Error al iniciar servo %d", i+1);
            return hardware_interface::CallbackReturn::ERROR;
        
        }

        rr_servo_set_state_operational(servo[i]);

        // Limpiamos trayectorias anteriores
        rr_clear_points_all(servo[i]);
    
        // Configurar cache
        rr_param_cache_setup_entry(servo[i], APP_PARAM_POSITION, true);
        rr_param_cache_setup_entry(servo[i], APP_PARAM_VELOCITY, true);
       


        // const float zero = 0;
        // int status = rr_set_zero_position (servo[i] ,zero);
        // if (status != RET_OK){RCLCPP_ERROR(rclcpp::get_logger("HwInterface"),"Error al fijar el 0 del servo %d: %d", i, status);}
        // else if (status == RET_OK){RCLCPP_ERROR(rclcpp::get_logger("HwInterface"),"SUCCES al fijar el 0 del servo %d: %d", i, status);}
    
        
    }

    

    // Inicializar servos
    for (int i = 0; i < 3; i++){
        
        rr_servo_set_state_operational(servo[i]);
       
    
    }

    
    

    //==========================DINAMIXEL===============================

    
    motorJ0 = dynamixelMotor("J0",3);
    motorJ1 = dynamixelMotor("J1",1);
    motorJ2 = dynamixelMotor("J2",5);
    
    // Garra
    motorJ3 = dynamixelMotor("J3",2);
    motorJ4 = dynamixelMotor("J4",12);
    
    // init comm and motor objects (port, protocol version, baudrate )
    if(dynamixelMotor::iniComm("/dev/ttyUSB0", 2, 57600.0f)){std::cout << "DMSXLS DETECTADOS" <<std::endl;}
    else{
        
        std::cout << "DMSXLS NO DETECTADOS, PROBANDO OTRA CONFIG" <<std::endl;
        if(dynamixelMotor::iniComm("/dev/ttyUSB1", 2, 57600.0f)){std::cout << "DMSXLS DETECTADOS CON LA OTRA CONFIG" <<std::endl;}
        else{std::cout << "DMSXLS NO DETECTADOS" <<std::endl;}
    
    }

    motorJ0.setControlTable();
    motorJ1.setControlTable();
    motorJ2.setControlTable();
    motorJ3.setControlTable();
    motorJ4.setControlTable();

    //motorJ0.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    motorJ0.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);
    motorJ1.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);
    motorJ2.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);
    motorJ3.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);
    motorJ4.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);

    
    torqueEnabled();



    return hardware_interface::CallbackReturn::SUCCESS;


}

std::vector<hardware_interface::StateInterface> HwInterface::export_state_interfaces(){
    
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (int i = 0; i < 8; i++){

        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);

        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);

        //state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &eff_[i]);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HwInterface::export_command_interfaces(){

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (int i = 0; i < 8; i++){

        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
    }

    return command_interfaces;
}

hardware_interface::return_type HwInterface::read(const rclcpp::Time & /*time*/,const rclcpp::Duration & /*period*/){

    
    

    float value;

    for (int i = 0; i < 3; i++){    

        rr_param_cache_update(servo[i]);

        rr_read_cached_parameter(servo[i], APP_PARAM_POSITION, &value);
        pos_[i] = deg2rad(value);

        //std::cout << "Posicion joint " << i << " " << pos_[i] <<std::endl;

        rr_read_cached_parameter(servo[i], APP_PARAM_VELOCITY, &value);
        vel_[i] = deg2rad(value);

    }
    
    pos_[0] += 1.56;
    pos_[1] -= 0.045;
    pos_[2] += 0.062;
    
    pos_[3] = deg2rad(motorJ0.getPresentPosition()) - 0.80;
    pos_[4] = deg2rad(motorJ1.getPresentPosition()) + 2.35;
    pos_[5] = (deg2rad(motorJ2.getPresentPosition()))/2 - 2.0; 
    pos_[6] = 0;
    pos_[7] = 0;

        
    vel_[3] = deg2rad(motorJ0.getPresentVelocity());
    vel_[4] = deg2rad(motorJ1.getPresentVelocity());
    vel_[5] = deg2rad(motorJ2.getPresentVelocity());
    vel_[6] = 0;
    vel_[7] = 0;



    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HwInterface::write(const rclcpp::Time & /*time*/ ,const rclcpp::Duration & period){

    
    
    for(int i = 0; i < 3; i++){

        target_vel_deg[i] = rad2deg(cmd_vel_[i]);

    }

    for(int i = 3; i < 8; i++){
       

        target_vel_deg[i] = rad2rpm(cmd_vel_[i]);

    }   


    
    for(int i = 0; i < 3; ++i){
               
        
        int status = rr_set_velocity(servo[i], target_vel_deg[i]);   

        if (status != RET_OK){RCLCPP_ERROR(rclcpp::get_logger("HwInterface"),"Error añadiendo punto servo %d: %d", i+1, status);}
        
    }
   

    motorJ0.setGoalVelocity(target_vel_deg[3]); 
    motorJ1.setGoalVelocity(target_vel_deg[4]);
    motorJ2.setGoalVelocity(target_vel_deg[5]);
    motorJ3.setGoalVelocity(target_vel_deg[7]);
    motorJ4.setGoalVelocity(target_vel_deg[6]);


    return hardware_interface::return_type::OK;
}

} // namespace hardware

PLUGINLIB_EXPORT_CLASS(hardware::HwInterface, hardware_interface::SystemInterface)
