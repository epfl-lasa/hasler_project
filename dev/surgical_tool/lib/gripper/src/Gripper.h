#ifndef PLATFORM_H
#define PLATFORM_H

#include "ros.h"
#include "mbed.h"
#include "SPI.h"
#include "QEC_1X_SPI.h"
#include "LP_Filter.h"
#include "MA_Filter.h"
#include "Gripper.h"
#include "definitions.h"
#include "custom_msgs_gripper/GripperInputMsg.h"
#include "custom_msgs_gripper/GripperOutputMsg.h"
#include "custom_msgs_gripper/gripperSetControllerSrv.h"
#include "custom_msgs_gripper/gripperSetStateSrv.h"
#include "PID_v1.h"
#include <Core>
#include <Dense>


class Gripper
{
  public:
    
    // ROS variables
    ros::NodeHandle _nh;
    //char _logMsg[256];
    bool _stop;

    GrippersId _gripperName;

    //Power Electronics Variables
    InterruptIn* _enable;

    bool _recoveringFromError;
    DigitalOut* _enableMotors;

    //Public Time
    uint32_t _timestamp;
    uint32_t _timestep;
    uint32_t _analogReadStamp;
    uint32_t _posSamplingStamp;
    uint32_t _speedSamplingStamp;
    uint32_t _accSamplingStamp;
    
    Timer _innerTimer; //! micros()
    uint64_t _innerCounterADC;

  public:
    Mutex _gripperMutex;

  public:

    // ROS variables  

      ros::Subscriber<custom_msgs_gripper::GripperInputMsg>*  _subGripperInput;
      ros::Publisher *_pubGripperOutput;
      custom_msgs_gripper::GripperOutputMsg _msgGripperOutput;
      ros::ServiceServer<custom_msgs_gripper::gripperSetStateSrvRequest,custom_msgs_gripper::gripperSetStateSrvResponse> *_servChangeState;
      ros::ServiceServer<custom_msgs_gripper::gripperSetControllerSrvRequest,custom_msgs_gripper::gripperSetControllerSrvResponse> *_servChangeCtrl;

      //CLIENT VARIABLES FROM (ROS)
        volatile float _ros_position;
        volatile float _ros_speed;

        volatile bool _ros_flagDefaultControl;
        volatile Controller _ros_controllerType;
        volatile State _ros_state;
        

        // State variables
        State _gripper_state;
        int8_t _gripper_controlledAxis;
        Controller _gripper_controllerType;

        volatile bool _flagClearLastState;
        volatile bool _flagControllerTypeChanged;
        bool _flagDefaultCtrlNew;
        volatile bool _flagCtrlGainsNew;
        volatile bool _flagInputReceived[NB_GI_CATEGORY];
        volatile bool _flagEmergencyCalled;
        
        bool _enterStateOnceFlag[NB_MACHINE_STATES];

        bool _gripper_flagDefaultControl;

        float _positionOffset; //! in m or radians
        float _positionD;
        float _positionD_filtered;
        float _positionCtrlOut;
        float _position;
        float _positionPrev;
        float _speed;
        float _speedPrev;
        float _acceleration;
        float _speedD;
        float _speedCtrlOut;
        float _pwmSpeedOut;
        LP_Filter _posDesiredFilter;
        LP_Filter _speedFilter;
        LP_Filter _accFilter;
        volatile uint _switchesState;
        bool _flagInWsConstrains;


        // Hardware variables
        PinName _csPin;
        QEC_1X *_encoder;
        PinName _motorPin;
        PwmOut *_motor;
        PinName _limitSwitchPin;

        InterruptIn *_limitSwitch;
        SPI *_spi;

        // PID variables
        // General Variables
        float _gripper_kpPosition;
        float _gripper_kiPosition;
        float _gripper_kdPosition;
        float _gripper_kpSpeed;
        float _gripper_kiSpeed;
        float _gripper_kdSpeed;

        volatile float _ros_kpPosition;
        volatile float _ros_kiPosition;
        volatile float _ros_kdPosition;
        volatile float _ros_kpSpeed;
        volatile float _ros_kiSpeed;
        volatile float _ros_kdSpeed;


        // PID

        PID *_pidPosition;
        PID *_pidSpeed;

        // Other variables

        static Gripper *me;

        uint32_t _toc;
        bool _tic; // flag for timer

        //*********************************************LIST-OF-METHODS**********
        //! Gripper_main.cpp
      public:
        Gripper(GrippersId name_);  //! 1
        ~Gripper(); //! 2
        void init(); //! 3
        void step(); //! 4
      private:
        //! GPIO Interruptions
        static void switchCallbackMotor();     //! 5
        //! ROS verification
        bool flagPositionInControl();
        bool flagSpeedInControl();
        //! Gripper_reset.cpp
      private:
        void softReset();   //! 2

        //! Gripper_ros.cpp
      public:
        void communicateToRos(); //! 1
        bool waitUntilRosConnect();

      private:
        // ROS
        static void
        updateGripperInput(const custom_msgs_gripper::GripperInputMsg &msg); //! 2
        static void updateState(const custom_msgs_gripper::gripperSetStateSrv::Request &req,
                                custom_msgs_gripper::gripperSetStateSrv::Response &resp); //! 3
        static void
        updateController(const custom_msgs_gripper::gripperSetControllerSrv::Request &req,
                         custom_msgs_gripper::gripperSetControllerSrv::Response &resp); //! 4
        void pubGripperOutput();                                            //! 5
      private:
        void updateGripperFromRos();

        //! Gripper_speed.cpp
      public:
        void setSpeed(float speed);  //! 2
      private:
        // PWM Computation for the DRIVERS
        void setPWM(float pwm); //! 3

        //! Gripper_utils.cpp
      public:
        float map(float x, float in_min, float in_max, float out_min,
                  float out_max); //! 1
        float clip(float x, float out_min, float out_max);
        float smoothRise(float x, float a, float b);
        float smoothFall(float x, float a, float b);
        float smoothRiseFall(float x, float a, float b, float c, float d);
                
        //!Gripper_sensors.cpp
        public : void getMotion(); //! 1
  private:
      //! Robot State
      void getPosition();                     //! 2
      void getSpeed();                    //! 3
      void getAcceleration();
      //! Estimate Robot Effort (ADC)    

  //! Gripper_emergency.cpp
  public:
    static void emergencyCallback();  //! 1
    static void switchCallback();
    void releaseGripper();           //! 2

  //! Gripper_control.cpp
  private:
      // Position and Speed control
      void posInterpolator();
      void positionControl();                           
      void speedControl();                              
      void gotoPoint(float point);                      
      void gotoPointGainsDefault();                     
      void speedPIDGainsDefault();                      
      void posCtrlLimitsSet();                          
      void speedCtrlLimitsSet();                        
      void loadDefaultPIDGains();
      void loadROSPIDGains();
      void setPIDGains();
      
      //! Gripper_clear.cpp
    private:
      //! Maintenance
      void positionReset();
      void limitSwitchesClear();
      void positionCtrlClear(); //! Put gains and set point to zero of the Position Control
      void speedCtrlClear(); //! Put gains and set point to zero of the Speed Control
      void clearLastState();
      void resetControllers(Controller controllerType);

};




#endif //PLATFORM_H
