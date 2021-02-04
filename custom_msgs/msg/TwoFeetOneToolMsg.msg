#TwoFeetOneToolMsg.msg
uint8 currentTool #current tool controlled by main platform, none=0, right=1, left=2, 
uint8 currentControlMode 
uint8 mixedPlatformStateMachine
float64[5] mixedPlatformOffset # platform main-> position/speed, platform supp -> speed
sensor_msgs/JointState mixedPlatformJointState
