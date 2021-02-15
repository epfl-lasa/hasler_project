# SurgicalTaskStateMsg.msg

int8 humanInputMode # DOMINANT_FOOT_TWO_ROBOTS = 0, SINGLE_FOOT_SINGLE_ROBOT = 1 
int8[2] controlPhase # INSERTION = 0, OPERATION = 1 
int8 currentRobot # LEFT = 0, RIGHT = 1
bool useTaskAdaptation
bool clutching
bool wait  