#ifndef __SURGICAL_TASK_H__
#define __SURGICAL_TASK_H__

#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include <termios.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "custom_msgs/FootInputMsg.h"
#include "custom_msgs/FootOutputMsg.h"
#include "custom_msgs_gripper/GripperOutputMsg.h"
#include "custom_msgs_gripper/GripperInputMsg.h"
#include "custom_msgs_gripper/SharedGraspingMsg.h"
#include "surgical_task/SurgicalTaskStateMsg.h"
#include "surgical_task/RobotStateMsg.h"
#include "surgical_task/TaskManagerStateMsg.h"
#include <dynamic_reconfigure/server.h>
#include "Eigen/Eigen"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "QpSolverRCM.h"
#include "QpSolverRCM3.h"
#include "QpSolverRCMCollision.h"
#include "QpSolverRCMCollision2.h"
#include "QpSolverRCMCollision3.h"
#include "CvxgenSolverRCM.h"
#include <pthread.h>
#include "custom_msgs/TwoFeetOneToolMsg.h"




#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define NB_ROBOTS 2
#define NB_DOF_FOOT_INTERFACE 5
#define TROCAR_SPACE_DIM 4
#define NB_AXES_JOYSTICK 8
#define MAX_ORIENTATION_ERROR 0.2f
#define NB_TRACKED_OBJECTS 4       // Number of objects tracked by the motion capture system (optitrack)
#define NB_OPTITRACK_SAMPLES 100     // Number of optitrack samples used for initial objects' pose estimation
#define EXTRA_DOF 4



class SurgicalTask 
{
  public: 

    enum Robot {LEFT = 0, RIGHT = 1};

    enum Tool {CAMERA = 0, RETRACTOR = 1};

    enum ControlPhase {AUTOMATIC_INSERTION = 0, OPERATION = 1, INSERTION = 2};

    enum ControlStrategy {PASSIVE_DS = 0, JOINT_IMPEDANCE = 1};

    enum FootDofs {FOOT_X = 0, FOOT_Y = 1, FOOT_PITCH = 2, FOOT_ROLL = 3, FOOT_YAW = 4};

    enum Mapping {POSITION_VELOCITY = 0, POSITION_POSITION = 1};

    enum TrocarSpaceVelocityDofs {V_UP = 0 , V_RIGHT = 1, V_INSERTION = 2, W_SELF_ROTATION = 3};

    enum TrocarSpacePositionDofs {X = 0 , Y = 1, Z = 2, SELF_ROTATION = 3};

    enum HumanInputDevice {JOYSTICK = 0, FOOT = 1};

    enum ObjectID {LEFT_ROBOT_BASIS = 0, RIGHT_ROBOT_BASIS = 1, LEFT_HUMAN_TOOL = 2, RIGHT_HUMAN_TOOL = 3};
    
    enum HumanInputMode {SINGLE_FOOT_SINGLE_ROBOT = 0, DOMINANT_INPUT_TWO_ROBOTS = 1};

    enum ToolsTracking {CAMERA_BASED = 0, OPTITRACK_BASED = 1};

    enum TaskJoystickButtons {X_BUTTON = 0, A_BUTTON = 1, IOS_BUTTON = 2, TRIANGLE_BUTTON = 3};

  private:
    
    ///////////////////
    // ROS variables //
    ///////////////////

    ros::NodeHandle _nh;
    ros::Rate _loopRate;
    float _dt;

    //////////////////////////////
    // Subscribers declarations //
    //////////////////////////////

    ros::Subscriber _subRobotPose[NB_ROBOTS];             // robot pose
    ros::Subscriber _subRobotTwist[NB_ROBOTS];            // robot twist
    ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];     // force torque sensor
    ros::Subscriber _subDampingMatrix[NB_ROBOTS];         // Damping matrix of DS-impedance controller
    ros::Subscriber _subFootInterfacePose[NB_ROBOTS];
    ros::Subscriber _subFootInterfaceWrench[NB_ROBOTS];
    ros::Subscriber _subFootOutput[NB_ROBOTS];
    ros::Subscriber _subFootInput[NB_ROBOTS];
    ros::Subscriber _subJoystick[NB_ROBOTS];
    ros::Subscriber _subCurrentJoints[NB_ROBOTS];
    ros::Subscriber _subOptitrackPose[NB_TRACKED_OBJECTS];  // Subscribe to optitrack markers' pose
    ros::Subscriber _subGripper;
    ros::Subscriber _subMarkersPosition;
    ros::Subscriber _subRobotExternalWrench[NB_ROBOTS];
    ros::Subscriber _subTaskJoystick;
    ros::Subscriber _subLegState[NB_ROBOTS];
    ros::Subscriber _subFootBaseWrench[NB_ROBOTS];
    ros::Subscriber _subFootHapticEfforts[NB_ROBOTS];
    ros::Subscriber _subFootInertiaCoriolisCompensation[NB_ROBOTS];
    ros::Subscriber _subLegCompensation[NB_ROBOTS];
    ros::Subscriber _subFootForceSensorModified[NB_ROBOTS];
    ros::Subscriber _subGripperAssistance;
    ros::Subscriber _subGripperFeedbackToPlatform;
    ros::Subscriber _subTaskManagerState;

    ///////////////////////////
    // Publisher declaration //
    ///////////////////////////

    ros::Publisher _pubDesiredTwist[NB_ROBOTS];           // Desired twist to DS-impdedance controller
    ros::Publisher _pubDesiredTask[NB_ROBOTS];           // Desired twist to DS-impdedance controller
    ros::Publisher _pubDesiredOrientation[NB_ROBOTS];     // Desired orientation to DS-impedance controller
    ros::Publisher _pubDesiredFootWrench[NB_ROBOTS];
    ros::Publisher _pubFootInput[NB_ROBOTS];
    ros::Publisher _pubToolToFootTorques[NB_ROBOTS];
    ros::Publisher _pubDesiredWrench[NB_ROBOTS];
    ros::Publisher _pubNullspaceCommand[NB_ROBOTS];
    ros::Publisher _pubDesiredJoints[NB_ROBOTS];
    ros::Publisher _pubGripper;
    ros::Publisher _pubRobotData[NB_ROBOTS];
    ros::Publisher _pubStiffness[NB_ROBOTS];
    ros::Publisher _pubSurgicalTaskState;
    ros::Publisher _pubRobotState[NB_ROBOTS];
    ros::Publisher _pubTwoFeetOneTool;
    ros::Publisher _pubCollisionSpheres;
    
    //////////////////////////
    // Messages declaration //
    //////////////////////////

    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Pose _msgDesiredPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;
    geometry_msgs::Wrench _msgDesiredFootWrench;
    custom_msgs::FootInputMsg _msgFootInput;
    custom_msgs::FootInputMsg _msgToolToFootTorques;
    std_msgs::Float32MultiArray _msgNullspaceCommand; 
    custom_msgs_gripper::GripperInputMsg _msgGripperInput;  
    custom_msgs_gripper::GripperOutputMsg _msgGripperOutput;  
    std_msgs::Float64MultiArray _msgStiffness;  
    std_msgs::Float64MultiArray _msgDesiredTask;  
    surgical_task::SurgicalTaskStateMsg _msgSurgicalTaskState;
    surgical_task::RobotStateMsg _msgRobotState;
    custom_msgs::TwoFeetOneToolMsg _msgTwoFeetOneTool;

    //////////////////////////
    // Tool characteristics //
    //////////////////////////

    float _toolMass[NB_ROBOTS];                             // Tool mass [kg]
    Eigen::Vector3f _toolOffsetFromEE[NB_ROBOTS];           // Tool offset along z axis of end effector [m]             
    Eigen::Vector3f _toolComPositionFromSensor[NB_ROBOTS];  // Offset of the tool [m] (3x1)
    Eigen::Vector3f _gravity;                               // Gravity vector [m/s^2] (3x1)  

    ////////////////////////////////
    // Tool/Robot state variables //
    ////////////////////////////////

    Eigen::Vector3f _x[NB_ROBOTS];                         // Tool tip position [m] (3x1)
    Eigen::Vector3f _xEE[NB_ROBOTS];                       // End-effector position [m] (3x1)
    Eigen::Vector4f _q[NB_ROBOTS];                         // Current quaternion (4x1)
    Eigen::Vector4f _qinit[NB_ROBOTS];                     // Initial quaternion (4x1)
    Eigen::Matrix3f _wRb[NB_ROBOTS];                       // Orientation matrix (3x1)
    Eigen::Matrix3f _wRb0[NB_ROBOTS];                       // Orientation matrix (3x1)
    Eigen::Vector3f _v[NB_ROBOTS];                         // Velocity [m/s] (3x1)
    Eigen::Vector3f _w[NB_ROBOTS];                         // Angular velocity [rad/s] (3x1)
    Eigen::VectorXf _currentJoints[NB_ROBOTS];             // Current joint positions [rad] (7x1)           
    Eigen::VectorXf _currentJointVelocities[NB_ROBOTS];    // Current joint velocities [rad/s] (7x1) 
    Eigen::VectorXf _currentJointTorques[NB_ROBOTS];       // Current joint torques [Nm] (7x1) 
    Eigen::Matrix<float,6,1> _wrench[NB_ROBOTS];           // Wrench [N and Nm] (6x1)
    Eigen::Matrix<float,6,1> _wrenchBias[NB_ROBOTS];       // Wrench bias [N and Nm] (6x1)
    Eigen::Matrix<float,6,1> _wrenchExtBias[NB_ROBOTS];    // Wrench bias [N and Nm] (6x1)
    Eigen::Matrix<float,6,1> _filteredWrench[NB_ROBOTS];   // Filtered wrench [N and Nm] (6x1)
    Eigen::Matrix3f _D[NB_ROBOTS];
    float _d1[NB_ROBOTS];
    Eigen::Vector3f _toolDir[NB_ROBOTS];
    Eigen::Vector3f _xdTool[NB_ROBOTS];
    Eigen::VectorXf _ikJoints[NB_ROBOTS];
    Eigen::Vector3f _xIK[NB_ROBOTS];                         // Position [m] (3x1)
    Eigen::Vector3f _xEEIK[NB_ROBOTS];                         // Position [m] (3x1)
    Eigen::Matrix3f _wRbIK[NB_ROBOTS];    
    Eigen::Vector3f _toolDirIK[NB_ROBOTS];
    float _dRCMTool[NB_ROBOTS];
    Eigen::Vector3f _trocarPosition[NB_ROBOTS];
    Eigen::Vector3f _trocarOrientation[NB_ROBOTS];
    Eigen::Vector3f _rEETrocar[NB_ROBOTS];
    Eigen::Vector3f _rEERCM[NB_ROBOTS];
    Eigen::Vector3f _xRCM[NB_ROBOTS];
    Eigen::Vector3f _Fext[NB_ROBOTS];
    Eigen::Matrix3f _wRRobotBasis[NB_ROBOTS];
    Eigen::Vector3f _rEECollision[NB_ROBOTS];
    Eigen::Vector3f _rToolCollision[NB_ROBOTS];
    Eigen::Vector3f _nToolCollision[NB_ROBOTS];
    float _dToolCollision[NB_ROBOTS];
    Eigen::Vector3f _nEECollision[NB_ROBOTS];
    float _dEECollision[NB_ROBOTS];
    Eigen::Vector3f _FmFoot[NB_ROBOTS];
    Eigen::Vector3f _Fm[NB_ROBOTS];
    Eigen::Vector3f _toolCollisionOffset[NB_ROBOTS];        // Tool correction [m] (3x1)
    float _depthGain[NB_ROBOTS];
    Eigen::Vector3f _toolTipCorrectionOffset[NB_ROBOTS];
    Eigen::Matrix3f _Roptitrack, _Rcamera;
    int _currentRobot;

    /////////////////////////////////
    // RobotTask/Command variables //
    /////////////////////////////////

    Eigen::Vector3f _xd[NB_ROBOTS];        // Desired position [m] (3x1)
    Eigen::Vector3f _xdEE[NB_ROBOTS];
    Eigen::Vector3f _xd0[NB_ROBOTS];        // Desired position [m] (3x1)
    Eigen::Vector4f _qd[NB_ROBOTS];        // Desired quaternion (4x1)
    Eigen::Vector3f _omegad[NB_ROBOTS];    // Desired angular velocity [rad/s] (3x1)
    Eigen::Vector3f _vd[NB_ROBOTS];        // Desired modulated DS [m/s] (3x1)
    Eigen::Vector3f _vdTool[NB_ROBOTS];        // Desired modulated DS [m/s] (3x1)
    Eigen::Vector3f _xRobotBaseOrigin[NB_ROBOTS];
    Eigen::Vector4f _qRobotBaseOrigin[NB_ROBOTS];
    float _selfRotationCommand[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _nullspaceWrench[NB_ROBOTS];
    Eigen::Matrix<float,7,1> _nullspaceCommand[NB_ROBOTS];
    Eigen::Vector3f _desiredOffsetPPM[NB_ROBOTS];
    float _desiredAnglePPM[NB_ROBOTS];
    float _desiredGripperPosition[NB_ROBOTS];
    std::vector<float> _insertionDistancePVM;
    Eigen::VectorXf _beliefs[NB_ROBOTS];
    Eigen::VectorXf _dbeliefs[NB_ROBOTS];
    Eigen::VectorXf _beliefsC;
    Eigen::VectorXf _dbeliefsC;
    Eigen::Vector3f _FdFoot[NB_ROBOTS];
    float _taud[NB_ROBOTS];
    Eigen::Vector3f _vHd[NB_ROBOTS];
    Eigen::Vector3f _vHRef[NB_ROBOTS];
    Eigen::Vector3f _vtRef[NB_ROBOTS];
    Eigen::Vector3f _vtd[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _humanClutchingOffset;
    Eigen::Vector4f _toolClutchingOffset;
    float _gripperClutchingOffset;
    float _humanGripperClutchingOffset;
    Eigen::Vector3f _vda;
    float _d = 0.07f;
    Eigen::Vector3f _Fh[NB_ROBOTS];
    float _gripperFeedback;
    int _taskId;

    //////////////////////////////
    // Foot interface variables //
    //////////////////////////////
    
    Eigen::Matrix<float,5,1> _footPose[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footPoseFiltered[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _trocarInput[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _oldTrocarInput[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footWrenchD[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footWrenchRef[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footWrenchM[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footTwist[NB_ROBOTS];
    int _footState[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footInputPosition[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footInputFilterAxisForce[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footInputKp[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footInputKd[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _desiredFootWrench[NB_ROBOTS];   // Filtered wrench [N and Nm] (6x1)
    Eigen::Matrix<float,5,1> _toolToFootTorques[NB_ROBOTS];   // Filtered wrench [N and Nm] (6x1)
    Eigen::Matrix<float,5,1> _footOffset[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _filterGainFootAxis[NB_ROBOTS];
    Eigen::Matrix<float,7,1> _legJointPositions[NB_ROBOTS];
    Eigen::Matrix<float,7,1> _legJointVelocities[NB_ROBOTS];
    Eigen::Matrix<float,7,1> _legJointTorques[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _legFootBaseWrench[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footHapticEfforts[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footWrenchModified[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _legCompensationTorques[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footInertiaCoriolisCompensationTorques[NB_ROBOTS];

    //////////////////////////////////
    // Configuration/Gain variables //
    ////////////////////////////////// 

    int _wrenchCount[NB_ROBOTS];
    int _wrenchExtCount[NB_ROBOTS];
    Eigen::Matrix<float,7,1> _stiffness[NB_ROBOTS];
    std::vector<int> _linearMapping;
    std::vector<int> _selfRotationMapping;
    ControlPhase _controlPhase[NB_ROBOTS];
    std::vector<int> _humanInputDevice;
    std::vector<int> _controlStrategy;
    std::vector<float> _footInterfaceRange[NB_ROBOTS];    
    std::vector<float> _footInterfaceMinDeadZone[NB_ROBOTS];
    std::vector<float> _footInterfaceMaxDeadZone[NB_ROBOTS];
    std::vector<float> _trocarSpaceVelocityGains;
    float _toolTipLinearVelocityLimit;
    float _toolTipSelfAngularVelocityLimit;
    float _trocarSpaceLinearDSFixedGain;
    float _trocarSpaceLinearDSGaussianGain;
    float _trocarSpaceLinearDSGaussianWidth;
    float _trocarSpaceSelfRotationGain;
    float _trocarSpaceSelfRotationRange;
    std::vector<float> _jointImpedanceStiffnessGain;
    float _safetyLimitsStiffnessGain;
    float _taskAdaptationAlignmentGain;
    float _taskAdaptationGaussianWidth;
    float _taskAdaptationConvergenceGain;
    float _taskAdaptationProximityGain;
    float _taskAdaptationExponentialGain;
    float _taskAdaptationOverallGain;
    float _gripperRange;
    int _dominantInputID, _nonDominantInputID;
    int _humanInputMode;
    int _toolsTracking;
    Eigen::Vector3f _insertionOffsetPPM[NB_ROBOTS];
    Eigen::Vector3f _operationOffsetRangePPM[NB_ROBOTS];
    Eigen::Vector3f _operationMinOffsetPVM[NB_ROBOTS];
    Eigen::Vector3f _operationMaxOffsetPVM[NB_ROBOTS];
    std::vector<float> _operationMinInsertion;
    float _eeSafetyCollisionDistance;
    float _eeSafetyCollisionRadius;
    float _toolSafetyCollisionDistance;
    float _toolSafetyCollisionRadius;
    int _switchingAxis;
    std::vector<float> _switchingThreshold;
    int _clutchingAxis;
    float _clutchingActivationThreshold;
    float _clutchingDeactivationThreshold;
    int _gripperControlAxis;
    Eigen::Matrix<float,5,5> _footPPMapping;
    Eigen::Matrix<float,5,5> _footPVMapping;
    float _taskAdaptationActivationThreshold;
    float _taskAdaptationDeactivationThreshold;
    Eigen::Matrix3f _eeCameraMapping;
    std::vector<int> _tool;
    std::vector<int> _humanInputID;
    float _markerFilterGain;
    float _eeLinearVelocityLimit;
    float _eeAngularVelocityLimit;
    float _linearForceFeedbackMagnitude;
    float _selfRotationTorqueFeedbackMagnitude;
    std::vector<float> _externalForcesDeadZones;
    float _filteredForceGain;   // Filtering gain for force/torque sensor
    int _sphericalTrocarId[NB_ROBOTS];
    Eigen::VectorXi _pillarsId;
    Eigen::MatrixXf _pillarsPosition;
    int _nbTasks;
    Eigen::MatrixXf _p[4];

    //////////////////////////////////////////////
    // Optitrack/Camera tool tracking variables //
    //////////////////////////////////////////////

    Eigen::MatrixXf _colorMarkersPosition;
    Eigen::MatrixXf _colorMarkersFilteredPosition;
    Eigen::MatrixXf _colorMarkersFilteredPosition2;
    Eigen::VectorXi _colorMarkersStatus;
    Eigen::Vector3f _humanToolPosition[2];
    int _humanToolStatus[2];
    Eigen::Matrix<float,3,NB_TRACKED_OBJECTS> _markersPosition;       // Markers position in optitrack frame
    Eigen::Matrix<float,4,NB_TRACKED_OBJECTS> _markersQuaternion;       // Markers position in optitrack frame
    Eigen::Matrix<float,3,NB_TRACKED_OBJECTS> _markersPosition0;      // Initial markers position in opittrack frame
    Eigen::Matrix<uint32_t,NB_TRACKED_OBJECTS,1> _markersSequenceID;  // Markers sequence ID
    Eigen::Matrix<uint16_t,NB_TRACKED_OBJECTS,1> _markersTracked;     // Markers tracked state
    uint32_t _optitrackCount;                                         // Counter used to pre-process the optitrack data
    Eigen::Vector3f _offsetTool;

    //////////////
    // Booleans //
    //////////////

    bool _debug;
    std::vector<bool> _useRobot;
    bool _useSim;
    bool _firstRobotPose[NB_ROBOTS];       // Monitor the first robot pose update
    bool _firstRobotTwist[NB_ROBOTS];      // Monitor the first robot twist update
    bool _stop;                            // Check for CTRL+C
    bool _firstRobotBaseFrame[NB_ROBOTS];
    bool _alignedWithTrocar[NB_ROBOTS];
    bool _trocarsRegistered[NB_ROBOTS];
    bool _firstWrenchReceived[NB_ROBOTS];       // Monitor first force/torque data update
    bool _wrenchBiasOK[NB_ROBOTS];              // Check if computation of force/torque sensor bias is OK
    bool _wrenchExtBiasOK[NB_ROBOTS];              // Check if computation of force/torque sensor bias is OK
    bool _firstDampingMatrix[NB_ROBOTS];        // Monitor first damping matrix update
    bool _firstHumanInput[NB_ROBOTS];
    bool _firstJointsUpdate[NB_ROBOTS];   
    bool _inputAlignedWithOrigin[NB_ROBOTS];
    bool _firstSphericalTrocarFrame[NB_ROBOTS];
    bool _firstPillarsFrame[3];
    bool _usePredefinedTrocars;
    bool _firstGripper;
    bool _firstFootSharedGrasping[NB_ROBOTS];
    bool _firstPublish[NB_ROBOTS];
    bool _firstColorMarkersPosition;
    bool _useFranka;
    Utils<float>::ROBOT_ID _robotID;
    bool _clutching;
    bool _trackingOK;
    bool _firstFootInput[NB_ROBOTS];
    float _tankH[NB_ROBOTS];
    float _alphaH[NB_ROBOTS];
    bool _useSafetyLimits;
    bool _allowTaskAdaptation;
    bool _useTaskAdaptation;
    bool _enableEECollisionAvoidance;
    bool _enableToolCollisionAvoidance;
    bool _enableWorkspaceCollisionAvoidance;
    std::vector<bool> _enablePhysicalHumanInteraction;
    std::vector<bool> _useFTSensor;
    bool _logData;
    bool _optitrackOK;
    bool _optitrackInitialized;
    bool _firstOptitrackPose[NB_TRACKED_OBJECTS];  // Monitor first optitrack markers update
    bool _switching = false;
    bool _wait = false;
    bool _insertionFinished[NB_ROBOTS];
    bool _allSubscribersOK = false;
    bool _allFramesOK = false;
    bool _taskAdaptation;
    bool _graspAssistanceOn;
    bool _taskStarted;
    bool _taskFinished;
    bool _firstTaskManagerState;

    /////////////////////
    // Other variables //
    /////////////////////

    std::string _fileName;
    std::ifstream _inputFile;
    std::ofstream _outputFile[5];
    tf::TransformListener _lr;
    tf::StampedTransform _transform;
    QpSolverRCM _qpSolverRCM[NB_ROBOTS];    
    QpSolverRCM3 _qpSolverRCM3[NB_ROBOTS];
    CvxgenSolverRCM _cvxgenSolverRCM;
    QpSolverRCMCollision* _qpSolverRCMCollision[NB_ROBOTS];   
    QpSolverRCMCollision::Result _qpResult[NB_ROBOTS];
    QpSolverRCMCollision2* _qpSolverRCMCollision2[NB_ROBOTS];    
    // QpSolverRCMCollision2::Result _qpResult[NB_ROBOTS];
    QpSolverRCMCollision3* _qpSolverRCMCollision3[NB_ROBOTS];    
    // QpSolverRCMCollision3::Result _qpResult[NB_ROBOTS];


    static SurgicalTask* me;


  public:

    // Class constructor
    SurgicalTask(ros::NodeHandle &n, double frequency, std::string fileName);

    // Initialize node
    bool init();

    // Run node
    void run();

  private:
    // Read parameters in configuration file
    bool readConfigurationParameters();

    // Initialize Subscribers and publishers
    void initializeSubscribersAndPublishers();

    // Initialize task parameters
    void initializeTaskParameters();

    // Callback called when CTRL is detected to stop the node
    static void stopNode(int sig);

    // Check that data are received from all the subscribers
    void checkAllSubscribers();

    // Check that we received all the frames (for simulation)
    void checkAllFrames();

    // Receive simulation frames
    void receiveFrames();

    // Main step function computing command for the two robots
    void step();

    // Process human input (joystick/foot)
    void processHumanInput();

    // Control step for robot r and human input h
    void robotControlStep(int r, int h);

    // Dominant foot two robots control human input mode
    void dominantFootTwoRobots();

    // Single foot single robot human input mode
    void singleFootSingleRobot();

    // Compute trocar input for robot r and human input h
    void computeTrocarInput(int r, int h);

    // Update the human tool pose if optitrack is used
    void updateHumanToolPosition();

    // Update robot r task state
    void updateRobotTaskState(int r);
    
    // Compute tool-tool collision point and distance
    void computeToolCollision(int r);

    // Update control phase of robot r
    void updateControlPhase(int r);

    // Perform an automatic insertion phase step for robot r and human input h
    void automaticInsertionStep(int r, int h);

    // Perform an operation phase step for robot r and human input h
    void operationStep(int r, int h);

    // Compute desired task specific tool tip velocity for robot r and human input h
    void computeDesiredToolVelocity(int r, int h);

    // Compute admittance velocity for robot r
    void computeAdmittanceVelocity(int r);

    // Scale input velocty to take into account fulcrum effect
    Eigen::Vector3f fulcrumEffectScaling(int r, Eigen::Vector3f vIn);

    void initializeBeliefs(int r);

    void taskAdaptation(int r, int h);
    
    // Log data to text file
    void logData();

    // Publish data to topics
    void publishData();
    
    // Callback to update the robot pose
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k);

    void updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg, int k); 

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

    void updateRobotExternalWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

    // Callback to update damping matrix form the DS-impedance controller
    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k); 

    void updateJoystick(const sensor_msgs::Joy::ConstPtr& msg, int k);

    void updateTaskJoystick(const sensor_msgs::Joy::ConstPtr& msg);

    void updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k);

    void updateFootInput(const custom_msgs::FootInputMsg::ConstPtr& msg, int k);
    
    void updateFootSharedGrasping(const custom_msgs_gripper::SharedGraspingMsg::ConstPtr& msg, int k);

    void updateGripperOutput(const custom_msgs_gripper::GripperOutputMsg::ConstPtr& msg);

    void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k); 

    void updateMarkersPosition(const std_msgs::Float64MultiArray::ConstPtr& msg); 

    void updateLegState(const sensor_msgs::JointState::ConstPtr& msg, int k);

    void updateFootBaseWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

    void updateFootHaptics(const custom_msgs::FootInputMsg::ConstPtr& msg, int k);

    void updateFootInertiaCoriolisCompensation(const custom_msgs::FootInputMsg::ConstPtr& msg, int k);

    void updateLegCompensation(const custom_msgs::FootInputMsg::ConstPtr& msg, int k);

    void updateFootForceSensorModified(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

    void updateGripperAssistance(const std_msgs::Bool::ConstPtr& msg);

    void updateGripperFeedbackToPlatform(const custom_msgs::FootInputMsg::ConstPtr& msg);

    void updateTaskManagerState(const surgical_task::TaskManagerStateMsg::ConstPtr& msg);

    uint16_t  checkTrackedMarker(float a, float b);

    void  optitrackInitialization();

    void footPositionMapping();

    void computeHapticFeedback(int r);

    void computeDesiredFootTorques(int r, int h);

    void registerTrocars();

    void getExpectedDesiredEETwist(int r, Eigen::Vector3f &vdEE, Eigen::Vector3f &omegadEE, Eigen::Vector3f vdk, Eigen::Vector3f rk);
};


#endif