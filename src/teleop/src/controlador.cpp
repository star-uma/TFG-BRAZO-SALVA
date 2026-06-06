#include "manipulador_teleop/controlador.hpp"


Controlador::Controlador() : Node("nodo_teleop")
{
    // PARAMETROS DE CONTROL
    control_rate_hz  = 50.0;  // Hz — debe ser >= publish_period del servo YAML
    linear_velocity  = 0.04;    // m/s
    angular_velocity = 0.04;    // rad/s
    joint_velocity   = 0.5;    // rad/s
    deadzone         = 0.2;
    garra = 0;

    // ESTADO INICIAL
    recibido         = false;
    modo_cartesiano  = false;   // arranca en modo cartesiano
    modo_orientacion = false;
    modo_operacion = 1;
    tiempo_boton = 0;
    

   
   
    // JOINTS
    joint_names_ = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8" };
    n_joints_    = static_cast<int>(joint_names_.size());



     // Inicializar KDL (lee robot_description del parameter server)
    if (!initKDL()) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo inicializar KDL. Abortando.");
    }



    //SUBSCRIBER JOY
    sub_joy_ = create_subscription<sensor_msgs::msg::Joy>("joy", 5,std::bind(&Controlador::joystick_callback, this, std::placeholders::_1));

    // JointState para mantener q_current_ actualizado
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,std::bind(&Controlador::jointStateCallback, this, std::placeholders::_1));

    // PUBLISHER
    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>("/brazo_controller/commands", 10);


    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_hz)),std::bind(&Controlador::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),"Contstructor listo");
}

Controlador::~Controlador()
{
    RCLCPP_INFO(get_logger(), "Nodo teleop destruido.");
    
    publishZeroVelocities();
}
 

bool Controlador::initKDL(){
    

    // Leer robot_description del parameter server
    declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
    std::string robot_description;
    
    get_parameter("robot_description", robot_description);
     
    // create kinematic chain
    KDL::Tree tree;
    kdl_parser::treeFromString(robot_description, tree);
    tree.getChain("base_link", "link7", chain_);
  
    
    unsigned int n = chain_.getNrOfJoints();

    q_current_.resize(n);
    KDL::SetToZero(q_current_);
 
    // Solver velocidad por pseudo-inversa  (eps=1e-5, maxiter=150)
    ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_wdls>(chain_, 1e-6);
   
    // Pesos: priorizar traslación sobre rotación
    Eigen::MatrixXd Wts = Eigen::MatrixXd::Identity(6,6);
    Wts(3,3) = 0.1;  // rx
    Wts(4,4) = 0.1;  // ry  
    Wts(5,5) = 0.1;  // rz
    ik_vel_solver_->setWeightTS(Wts);

  
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
 

   




    RCLCPP_INFO(this->get_logger(),"INIT KDL LISTO");

    return true;
}

void Controlador::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
   
    // Mapear por nombre para no depender del orden del mensaje
    for (int i = 0; i < chain_.getNrOfJoints(); ++i) {

        for (size_t j = 0; j < msg->name.size(); ++j) {
            
            if (msg->name[j] == joint_names_[i]) {
                
                //RCLCPP_INFO_STREAM(this->get_logger(),"JOINT "<< j <<" se llama: "<<msg->name[j]);
                

                    
                // RCLCPP_INFO_STREAM(this->get_logger(),"Posicion JOINT "<< i <<" es: "<<q_current_(i));
                
                //RCLCPP_INFO_STREAM(this->get_logger(),"Su offset es: "<<offset[i]); 
                
                q_current_(i) = msg->position[j];
                 
                
                
                break;
            }
        }
    }  

      
}

void Controlador::timerCallback(){

    tiempo_boton++;
    if(tiempo_boton > 1000000000){tiempo_boton = 0;}

    if (!recibido)            return;
    if (!joy_msg_.has_value()) return;
    if (joy_msg_->axes.size() < 4) return;
  
    
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.resize(n_joints_, 0.0);

    
    //=================== MODO CARTESIANO  ======================
    
    // Mapeo REAL de ejes del joystick:
    //  axes[0]: Alabeo (+1 izquierda, -1 derecha)
    //  axes[1]: Cabeceo (1 arriba, -1 abajo)
    //  axes[2]: Guiñada(+1 izquierda, -1 derecha)
    //  axes[3]: Potenciómetro

    // Convención de ejes (joystick):
    //  axes[1]: Cabeceo =  traslación X  (adelante/atrás)
    //  axes[0]: Alabeo  = traslación Y  (izq/der)
    //  axes[2]: Guiñada = traslación Z  (giro izda/dcha)
    
  
    // =========================== MODO CART =====================

    if (modo_cartesiano){

        const double dt = 1.0 / control_rate_hz;

        // Resolver IK de velocidades 
        KDL::JntArray q_dot(chain_.getNrOfJoints());

        twist.vel = KDL::Vector::Zero();
        twist.rot = KDL::Vector::Zero();


        // ========================= INTEGRAR JOYSTICK =================

        if(!modo_orientacion){

            if (std::abs(joy_msg_->axes[1]) > deadzone){twist.vel.x(joy_msg_->axes[1] * linear_velocity);}

            if (std::abs(joy_msg_->axes[0]) > deadzone){twist.vel.y(joy_msg_->axes[0] * linear_velocity);}

            if (std::abs(joy_msg_->axes[3]) > deadzone){twist.vel.z(joy_msg_->axes[3] * linear_velocity);}

            if (twist.vel.Norm() < 1e-6) { publishZeroVelocities(); return; }

        }
        else{

           
            if (std::abs(joy_msg_->axes[1]) > deadzone){twist.rot.x(joy_msg_->axes[1] * angular_velocity);}

            if (std::abs(joy_msg_->axes[0]) > deadzone){twist.rot.y(joy_msg_->axes[0] * angular_velocity);}

            if (std::abs(joy_msg_->axes[3]) > deadzone){twist.rot.z(joy_msg_->axes[3] * angular_velocity);}

            if (twist.rot.Norm() < 1e-6) { publishZeroVelocities(); return; }

        }   

        
        // ======================== IK velocidad =============================
      
           
        if (twist.vel.Norm() < 1e-6 && twist.rot.Norm() < 1e-6) { publishZeroVelocities(); return; }

        if (modo_operacion == 2) // TCP -> transformar twist al frame base
        {
            KDL::Frame x;
            if (fk_solver_->JntToCart(q_current_, x) < 0) { publishZeroVelocities(); RCLCPP_WARN(this->get_logger(), "FK falló"); return; }
            twist = x.M * twist;
        }

        if (ik_vel_solver_->CartToJnt(q_current_, twist, q_dot) < 0) { publishZeroVelocities(); RCLCPP_WARN(this->get_logger(), "IK falló"); return; }
    

        


        //====================== SATURACIÓN ==========================
        for (int i = 0; i < chain_.getNrOfJoints(); ++i) {        

            cmd_msg.data[i] = std::clamp(q_dot(i), -joint_velocity, joint_velocity);
    
        }

    }

       
    // =========================== MODO JOINT =====================

    else{
       
           
        if(!modo_orientacion){

            if (std::abs(joy_msg_->axes[0]) > deadzone){cmd_msg.data[0] = -joy_msg_->axes[0] * linear_velocity;}

            if (std::abs(joy_msg_->axes[1]) > deadzone){cmd_msg.data[1] = -joy_msg_->axes[1] * linear_velocity;}

            if (std::abs(joy_msg_->axes[3]) > deadzone){cmd_msg.data[2] = -joy_msg_->axes[3] * linear_velocity;}
        }
        else{

            if (std::abs(joy_msg_->axes[1]) > deadzone){cmd_msg.data[3] = joy_msg_->axes[1] * angular_velocity;}

            if (std::abs(joy_msg_->axes[0]) > deadzone){cmd_msg.data[4] = joy_msg_->axes[0] * angular_velocity;}

            if (std::abs(joy_msg_->axes[3]) > deadzone){cmd_msg.data[5] = -joy_msg_->axes[3] * angular_velocity;}

        }

        
    }

    //============================ GARRA ===========================

    if(garra == 1){

        cmd_msg.data[6] = 3;
        cmd_msg.data[7] = 3;
    }
    else if(garra == -1){

        cmd_msg.data[6] = -3;
        cmd_msg.data[7] = -3;

    }
    else{

        cmd_msg.data[6] = 0;
        cmd_msg.data[7] = 0;

    }


    
   
    pub_cmd_->publish(cmd_msg);

    
}

void Controlador::joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
    
    recibido = true;
    joy_msg_  = *msg;

    if (msg->buttons.size() < 1) return;
    
        // Aumentar velocidad lineal
        if (msg->buttons[6] == 1){

            linear_velocity  += 0.01;
            
          
            RCLCPP_INFO_STREAM(this->get_logger(), "+Linear velocity: "<< linear_velocity);
        }
        
        // Disminuir velocidad lineal
        if (msg->buttons[4] == 1){

            linear_velocity  -= 0.01;
          
            RCLCPP_INFO_STREAM(this->get_logger(), "-Linear velocity: "<< linear_velocity);
        }

         // Aumentar velocidad angular
        if (msg->buttons[7] == 1){

            angular_velocity += 0.02;

            RCLCPP_INFO_STREAM(this->get_logger(), "+Angular velocity: "<< angular_velocity);
        }
        
        // Disminuir velocidad angular
        if (msg->buttons[9] == 1){

            angular_velocity -= 0.02;
            
            RCLCPP_INFO_STREAM(this->get_logger(), "-Angular velocity: "<< angular_velocity);
        }

        
        //------------------------------- GARRA ---------------------

        
        if (msg->buttons[13] == 1){

            garra = -1;
            
            RCLCPP_INFO_STREAM(this->get_logger(), "Cerrando GARRA ");
        }

        if (msg->buttons[14] == 1){

            garra = 0;
            
            RCLCPP_INFO_STREAM(this->get_logger(), "Parando GARRA ");
        }
        
        if (msg->buttons[15] == 1){

            garra = 1;
            
            RCLCPP_INFO_STREAM(this->get_logger(), "Abriendo GARRA ");
        }
        
       
        //------------------------------------------------------------

        // Botón 4 (índice 3): Toggle Posición / Orientación
        if (msg->buttons[1] == 1){

            if(tiempo_boton >= 1 * control_rate_hz){

                tiempo_boton = 0;

                modo_orientacion = !modo_orientacion;

                if(modo_orientacion){

                    // Pesos: priorizar rotación sobre traslación
                    Eigen::MatrixXd Wts = Eigen::MatrixXd::Identity(6,6);
                    Wts(0,0) = 0.1;  // lx
                    Wts(1,1) = 0.1;  // ly  
                    Wts(2,2) = 0.1;  // lz
                    ik_vel_solver_->setWeightTS(Wts);

                    RCLCPP_INFO(this->get_logger(), "Paso a control de orientación");
                
                }
                else if(!modo_orientacion){

                    // Pesos: priorizar traslación sobre rotación
                    Eigen::MatrixXd Wts = Eigen::MatrixXd::Identity(6,6);
                    Wts(3,3) = 0.1;  // rx
                    Wts(4,4) = 0.1;  // ry  
                    Wts(5,5) = 0.1;  // rz
                    ik_vel_solver_->setWeightTS(Wts);
                    
                    RCLCPP_INFO(this->get_logger(), "Paso a control de posición");
                
                }
                
            }
        }

        if (msg->buttons[3] == 1){

            if(tiempo_boton >= 1 * control_rate_hz){

                tiempo_boton = 0;
                
                modo_cartesiano = !modo_cartesiano;
                if(modo_cartesiano){RCLCPP_INFO(this->get_logger(), "Paso a control cartesiano");}
                else{RCLCPP_INFO(this->get_logger(), "Paso a control articular");}
            }
        }
        

        // MODO OPERACION

        if (msg->buttons[11] == 1){
                
            modo_operacion = 2;
            RCLCPP_INFO(this->get_logger(), "MODO OPERACIÓN TCP");
            
        }

        if (msg->buttons[12] == 1){
                
            modo_operacion = 1;
            RCLCPP_INFO(this->get_logger(), "MODO OPERACIÓN WORLD");
            
        }

}

void Controlador::publishZeroVelocities()
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(n_joints_, 0.0);


    if(garra == 1){

        msg.data[6] = 3;
        msg.data[7] = 3;
    }
    else if(garra == -1){

        msg.data[6] = -3;
        msg.data[7] = -3;

    }
    else{

        msg.data[6] = 0;
        msg.data[7] = 0;

    }

    pub_cmd_->publish(msg);

}


