#include "mainpp.h"

#include "dwt_stm32_delay.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "Gripper.h"
#include "definitions.h"


#define ListofGrippers(enumeration, names) names,
extern char const *Grippers_Names[]{
    GRIPPER_LIST};
#undef ListofAxes

Gripper gripper(RIGHT_GRIPPER);
Ticker doCommFlipper;
rtos::Thread th_Control;

volatile bool flag_doComm;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  gripper._nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  gripper._nh.getHardware()->reset_rbuf();
}

void doCommCb()
{
  flag_doComm=true;
}

void doControlTH() {
  while (!gripper._stop) {
   // gripper.step();
  }
}

void setup(void)
{
  //gripper.init();
  gripper._nh.initNode();
  flag_doComm=false;
  doCommFlipper.attach_us(&doCommCb,1500);
  th_Control.start(doControlTH);
}

void loop(void)
{
  if (flag_doComm) {
    if (gripper.waitUntilRosConnect())
    { 
        gripper.communicateToRos();
    }   
    flag_doComm=false;
  }
}

