/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include <PID_v1.h>



/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(Timer* timer_, float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, int POn, int ControllerDirection, float filterGain)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    myTimer = timer_;
    inAuto = false;

   // PID::setOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 100 microseconds

    PID::setControllerDirection(ControllerDirection);

    PID::setTunings(Kp, Ki, Kd, POn);

    lastTime = myTimer->read_us()-SampleTime;

    dInputFilter.setAlpha(filterGain);
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(Timer *timer_, float *Input, float *Output, float *Setpoint,
         float Kp, float Ki, float Kd, int ControllerDirection, float filterGain)
    : PID::PID(timer_, Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection, filterGain)
{

}

PID::PID(Timer *timer_, float *Input, float *Output, float *Setpoint,
         float Kp, float Ki, float Kd, int ControllerDirection)
    : PID::PID(timer_, Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection, 0.5f)
{
}

PID::~PID()
{
   delete(myOutput);
   delete(myTimer);
   delete(myInput);
   delete(mySetpoint);
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::compute()
{
   if(!inAuto) return false;
   unsigned long now = myTimer->read_us();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      float input = *myInput;
      float error = *mySetpoint - input;
      
      float dInput = dInputFilter.update(input - lastInput);
      outputSum += (ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pOnE) outputSum-= kp * dInput;

      /*Add Proportional on Error, if P_ON_E is specified*/
	   float output;
      if(pOnE) output = kp * error;
      else output = 0;

      /*Compute Rest of PID Output*/
      output += outputSum - kd * dInput;


      if (output >= outMax)
      {
         if (ki!=0.0f)
         {
            outputSum -= output - outMax;
         }
         else
         {
            outputSum = 0.0f;
         }
         
         output = outMax;
      }
      else if (output <= outMin)
      {
         if (ki != 0.0)
         {
            outputSum -= output - outMin;
         }
         else
         {
            outputSum = 0.0f;
         }
         output = outMin;
      }
      
      if(outputSum > outMax) outputSum = outSumMax;
	   else if(outputSum < outMin) outputSum = outSumMin;

	   *myOutput = output; 

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
      errorM = error;
   }
   else return false;
   
   return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::setTunings(float Kp, float Ki, float Kd, int POn)
{
   // float Ki=Kp/Ti;
   // float Kd=Kp*Td;

   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   float SampleTimeInSec = ((float)SampleTime)/1000000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::setTunings(float Kp, float Ki, float Kd){
    setTunings(Kp, Ki, Kd, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in micros, at which the calculation is performed
 ******************************************************************************/
void PID::setSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = NewSampleTime/SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clip it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::setOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
   outSumMax = 0.9f*outMax;
   outSumMin = 0.9f*outMin;


   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(outputSum > outSumMax) outputSum= outSumMax;
	   else if(outputSum < outSumMin) outputSum= outSumMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::setMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outSumMax) outputSum = outSumMax;
   else if(outputSum < outSumMin) outputSum = outSumMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::setControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID::getKp(){ return  dispKp; }
float PID::getKi(){ return  dispKi;}
float PID::getKd(){ return  dispKd;}
float PID::getIntegralTerm(){return outputSum;}
float PID::getError() {return errorM;}
int PID::getMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::getDirection(){ return controllerDirection;}

void PID::reset() {
   lastInput=*myInput;
   errorM=*mySetpoint - *myInput;
   outputSum=0.0;
}