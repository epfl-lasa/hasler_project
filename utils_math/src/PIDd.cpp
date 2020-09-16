/**********************************************************************************************
 * Arduino PIDd Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include "PIDd.h"



/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PIDd::PIDd(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int POn, int ControllerDirection, double filterGain)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PIDd::setOutputLimits(0, 255);				//default output limit corresponds to
												         //the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 100 milliseconds
    SampleTimeSec = SampleTime * 0.001; 
    PIDd::setControllerDirection(ControllerDirection);

    PIDd::setTunings(Kp, Ki, Kd, POn);

   lastTime = ros::Time::now().toSec() - SampleTime*0.001;
   // lastTime = myTimer->read_us()-SampleTime;

    dInputFilter.setAlpha(filterGain);
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PIDd::PIDd(double *Input, double *Output, double *Setpoint,
         double Kp, double Ki, double Kd, int ControllerDirection, double filterGain)
    : PIDd::PIDd(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection, filterGain)
{

}

PIDd::PIDd(double *Input, double *Output, double *Setpoint,
         double Kp, double Ki, double Kd, int ControllerDirection)
    : PIDd::PIDd(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection, 0.0f)
{
}

PIDd::~PIDd()
{
   delete(myOutput);
   delete(myInput);
   delete(mySetpoint);
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pidd Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PIDd::compute()
{
   if(!inAuto) return false;
   ros::Time now = ros::Time::now();
   double timeChange = now.toSec() - lastTime;

   if(timeChange>= (double) SampleTime*0.001)
   {
      /*Compute all the working error variables*/
      double input = *myInput;
      double error = *mySetpoint - input;
      double dInput = dInputFilter.update(input - lastInput);
      outputSum += (ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pOnE) outputSum-= kp * dInput;

       if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
	   double output;
      if(pOnE) output = kp * error;
      else output = 0;

      /*Compute Rest of PIDd Output*/
      output += outputSum - kd * dInput;


      // if (output >= outMax)
      // {
      //    if (ki!=0.0f)
      //    {
      //       outputSum -= output - outMax;
      //    }
      //    else
      //    {
      //       outputSum = 0.0f;
      //    }
         
      //    output = outMax;
      // }
      // else if (output <= outMin)
      // {
      //    if (ki != 0.0)
      //    {
      //       outputSum -= output - outMin;
      //    }
      //    else
      //    {
      //       outputSum = 0.0f;
      //    }
      //    output = outMin;
      // }
      
      if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;

	   *myOutput = output; 

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now.toSec();
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
void PIDd::setTunings(double Kp, double Ki, double Kd, int POn)
{
   // double Ki=Kp/Ti;
   // double Kd=Kp*Td;

   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime) * 0.001;
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
void PIDd::setTunings(double Kp, double Ki, double Kd){
    setTunings(Kp, Ki, Kd, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in micros, at which the calculation is performed
 ******************************************************************************/
void PIDd::setSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = NewSampleTime/SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (int)NewSampleTime;
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
void PIDd::setOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;


   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PIDd::setMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PIDd::initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PIDd::initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PIDd will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PIDd::setControllerDirection(int Direction)
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
 * functions query the internal state of the PIDd.  they're here for display
 * purposes.  this are the functions the PIDd Front-end uses for example
 ******************************************************************************/
double PIDd::getKp(){ return  dispKp; }
double PIDd::getKi(){ return  dispKi;}
double PIDd::getKd(){ return  dispKd;}
double PIDd::getIntegralTerm(){return outputSum;}
double PIDd::getError() {return errorM;}
int PIDd::getMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PIDd::getDirection(){ return controllerDirection;}

void PIDd::reset() {
   lastInput=*myInput;
   errorM=*mySetpoint - *myInput;
   outputSum=0.0;
}