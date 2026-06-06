#ifndef DYNAMIXEL_ROS2_H
#define DYNAMIXEL_ROS2_H

// DEPENDENCIES
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <string>

// CLASS DEFINITION
class dynamixelMotor
{
    public:
        // CONSTS
        static const int CURRENT_CONTROL_MODE = 0, 
                         VELOCITY_CONTROL_MODE = 1, 
                         POSITION_CONTROL_MODE = 3, 
                         EXTENDED_POSITION_CONTROL_MODE = 4, 
                         CURRENT_BASED_POSITION_CONTROL = 5, 
                         PWM_CONTROL_MODE = 16;

        // CONSTRUCTOR AND DESTRUCTOR
        dynamixelMotor();
        dynamixelMotor(std::string IDENTIFICATOR, int ID);
        ~dynamixelMotor();

        // COMMUNICATION START
        static bool iniComm(char* PORT_NAME, float PROTOCOL_VERSION, int BAUDRATE);

        // TABLE INI
        void setControlTable();

        // GETTERS AND SETTERS
        int getID();
        void setID(int NEW_ID);

        std::string getModel();

        int getBaudrate();
        void setBaudrate();

        int getReturnDelayTime();
        void setReturnDelayTime(int RETURN_DELAY_TIME);

        std::string getOperatingMode();
        void setOperatingMode(int MODE);

        int getShadowID();
        void setShadowID(int NEW_SH_ID);

        int getProcotolType();
        // void setProtocolType(int PROTOCOL_TYPE);  not implemented at the moment

        int getHomingOffset(); // returns degrees
        void setHomingOffset(int DEGREES);

        double getMovingThreshold();
        void setMovingThreshold(double RPM);

        int getTempLimit();
        void setTempLimit(int TEMPERATURE);

        float getMaxVoltageLimit();
        void setMaxVoltageLimit(float MAX_VOLTAGE);

        float getMinVoltageLimit();
        void setMinVoltageLimit(float MIN_VOLTAGE);

        int getPWMLimit();
        void setPWMLimit(int PWM);

        float getCurrentLimit();
        void setCurrentLimit(float CURRENT_mA);
        
        float getVelLimit();
        void setVelLimit(float VEL_LIMIT_RPM);

        float getMaxPosLimit();
        void setMaxPosLimit(float MAX_POS_LIMIT_DEGREES);
        
        float getMinPosLimit();
        void setMinPosLimit(float MIN_POS_LIMIT_DEGREES);

        bool getTorqueState();
        void setTorqueState(bool TORQUE_ENABLE);

        bool getLedState();
        void setLedState(bool LED_STATE);

        int getStatusReturnLevel();
        void setStatusReturnLevel(int STATUS_RETURN_LEVEL);

        std::vector<bool> getHardwareErrorStatus();

        void getVelocityPIValues(int &P, int &I);
        void setVelocityPIValues(int P, int I);

        void getPositionPIDValues(int &P, int &I, int &D);
        void setPositionPIDValues(int P, int I, int D);

        void getFeedforwardGains(int &FFG1, int &FFG2);
        void setFeedforwardGains(int FFG1, int FFG2);

        int getBusWatchdog();
        void setBusWatchdog(int BUS_WATCHDOG_VALUE);

        int getGoalPWM();
        void setGoalPWM(int GOAL_PWM);

        int getGoalCurrent();
        void setGoalCurrent(int GOAL_CURRENT);

        double getGoalVelocity();
        void setGoalVelocity(double GOAL_VELOCITY);

        double getProfileAcceleration();
        void setProfileAcceleration(double PROFILE_ACCELERATION);

        double getProfileVelocity();
        void setProfileVelocity(double PROFILE_VELOCITY);

        double getGoalPosition();
        void setGoalPosition(double GOAL_POSITION);

        double getRealtimeTick();

        bool isMoving();

        int getPresentPWM();

        int getPresentCurrent();

        double getPresentVelocity();

        double getPresentPosition();

        float getPresentInputV();

        int getPresentTemperature();

        bool getBackupReady();

        // CONFIG METHODS
        void configDriveMode(bool REVERSE_MODE, bool SLAVE_MODE, bool TIME_BASED_PROFILE, bool TORQUE_AUTO_ON);
        void showDriveModeConfig();

        void configStartup(bool TORQUE_ON, bool RAM_RESTORE);
        void showStartupConfig();

        void configShutdown(bool INPUT_VOLTAGE_ERROR, bool OVERHEATING_ERROR, bool ENCODER_ERROR, bool ELECTRICAL_SHOCK_ERROR, bool OVERLOAD_ERROR);
        void showShutdownConfig();
        
    private:
        // MAPS USED TO CHANGE BETWEEN DIFF EEPROM CONTROL TABLES
        static std::map<std::string, int> ADDR_DMXL22, ADDR_DMXL25;

        // MAPS THAT CONECTS A MODELS WITH ITS MODEL NUMBERS
        static std::map<int, std::string> DMXL_MODELS;

        // Dynamixels parameters
        int ID;
        int MODEL;
        std::string IDENTIFICATOR;
        std::map<std::string, int> CONTROL_TABLE;
};

#endif