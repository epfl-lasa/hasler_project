# FootHapticDataMsg.msg

#Leg
float32[7] fh_haptEffLegIn #the desired input haptic effort projected in the leg's joint space
#float32[7] fh_normEffortCoeffs #normalized (sum{coeff_joints} = 1.0) coefficients in the joint space of the leg, that are calculated by projecting the desired haptic cues efforts in the joints of the leg and normalizing. These coefficients reveal the sensitivity of the leg joint ot the haptic cue, informing the influence of the joint on the sensation. 
float32[7] fh_jointLimCoeffs #coefficients that reveal how close the joint in the leg is to joint limits. 
float32[7] fh_bckgndEffLeg #the addition of the gravity torques + the virtual impedance torques rendered in the platform, and projected to the leg's joint space
#float32[7] fh_weberRatios #these are the ratio between the projected effort of the haptic cue in the joint of the leg divided by the background effort (gravity effort + virtual impedance)
#float32[7] fh_weberCoeff#these coefficients aim at correcting the current weberRatios by calculating how much should a haptic effort in the joint of the leg 

#Platform

float32[5] fh_haptEffLPF #the low frequencies of the haptic cues feedback
float32[5] fh_haptEffLPF_Proj #the low frequencies of the haptic cues projected on the pre-existing torque in the platform
float32[5] fh_haptEffHPF #the high pass frequencies of the haptic cues
float32[5] fh_effFootOut #the total output (amplified) haptic cues send to the platform 
float32[5] fh_vibFB #the artificial event based vibration added to the haptic cues
float32[5] fh_maxPossGains #the maximum possible gains in each joint of the platform, given its effort limits to avoid saturation

float32 fh_effGainRaw #this is the gain calculated for all platform joint, derived from the weighted average of coefficicients above in the leg's joint space
float32 fh_effGainBounded #these are the gains bounded by the minimum of the maximum possible gains


