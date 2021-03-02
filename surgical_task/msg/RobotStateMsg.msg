# RobotStateMsg.msg

int8 controlPhase
int8 linearMapping
int8 selfRotationMapping
float32[3] trocarPosition
float32[3] tipPosition
float32[3] ikTipPosition
float32 dRcmTrocar
float32 dRcmTip
float32[7] currentJoints
float32[7] ikJoints
int8 ikRes
float32[5] humanInput
float32[3] vdTool
float32 selfRotationCommand
float32[3] currentOffsetFromInsertion
float32[3] ikOffsetFromInsertion
float32[3] desiredOffsetFromInsertion
bool eeCollisionConstraintActive
float32 dEEEE 
bool toolCollisionConstraintActive
float32 dToolTool
bool workspaceCollisionConstraintActive
float32 desiredGripperPosition

