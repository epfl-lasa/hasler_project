# SurgicalTaskStateMsg.msg

int8 humanInputMode # SINGLE_FOOT_SINGLE_ROBOT = 0, DOMINANT_FOOT_TWO_ROBOTS = 1  
int8 currentRobot # LEFT = 0, RIGHT = 1
bool useTaskAdaptation
float32[3] beliefsC
bool clutching
bool wait  