#TwoFeetOneToolMsg.msg
uint8 currentTool #current tool controlled by main platform, none=0, right=1, left=2, 
uint8 currentControlMode #position, speed
uint8 mixedPlatformStateMachine #{BOTH_TOOLS_DISABLED, RIGHT_TOOL_ENABLED, LEFT_TOOL_ENABLED, WAITING_TOOL_TO_CHANGE, STANDBY, CALCULATE_OFFSET_MIXED_PLATFORM};
float64[5] mixedPlatformOffset # platform main-> position/speed, platform supp -> speed
sensor_msgs/JointState mixedPlatformJointState
