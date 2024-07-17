/**********************************************************************************************
 * Arduino PID Library - Version 2.0.0
 * by Robin Perdreau
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_v2.hpp"
#include "utils/utils.hpp"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
         double Kp, double Ki, double Kd, int POn, int ControllerDirection) {

   m_myOutput = Output;
   m_myInput = Input;
   m_mySetpoint = Setpoint;
   m_inAuto = false;

   // default output limit corresponds to the arduino pwm limits
   PID::SetOutputLimits(0, 255);

   // default Controller Sample Time is 0.1 seconds
   m_sampleTime = 100; 

   PID::SetControllerDirection(ControllerDirection);
   PID::SetTunings(Kp, Ki, Kd, POn);

   m_lastTime = millis() - m_sampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
         double Kp, double Ki, double Kd, int ControllerDirection)
    : PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection) {}

/* WrapCompute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done. With wrapping :DDDDDD
 *   lol
 **********************************************************************************/
bool PID::WrapCompute() {
   if (!m_inAuto)
      return false;

   unsigned long now = millis();
   unsigned long timeChange = (now - m_lastTime);

   if (timeChange >= m_sampleTime) {
      /* Compute all the working error variables */
      double input = *m_myInput;
      double error = heading_error(*m_mySetpoint, input);

      double dInput = (input - m_lastInput);
      m_outputSum += (m_ki * error);

      /* Add Proportional on Measurement, if P_ON_M is specified */
      if (!m_pOnE)
         m_outputSum -= m_kp * dInput;

      if (m_outputSum > m_outMax)
         m_outputSum = m_outMax;
      else if (m_outputSum < m_outMin)
         m_outputSum = m_outMin;

      /* Add Proportional on Error, if P_ON_E is specified */
      double output;
      if (m_pOnE)
         output = m_kp * error;
      else
         output = 0;

      /* Compute Rest of PID Output */
      output += m_outputSum - m_kd * dInput;

      if (output > m_outMax)
         output = m_outMax;
      else if (output < m_outMin)
         output = m_outMin;
      *m_myOutput = output;

      /* Remember some variables for next time */
      m_lastInput = input;
      m_lastTime = now;
      return true;
   }
   else
      return false;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute() {
   if (!m_inAuto)
      return false;

   unsigned long now = millis();
   unsigned long timeChange = (now - m_lastTime);

   if (timeChange >= m_sampleTime) {
      /* Compute all the working error variables */
      double input = *m_myInput;
      double error = *m_mySetpoint - input;
      double dInput = (input - m_lastInput);
      m_outputSum += (m_ki * error);

      /* Add Proportional on Measurement, if P_ON_M is specified */
      if (!m_pOnE)
         m_outputSum -= m_kp * dInput;

      if (m_outputSum > m_outMax)
         m_outputSum = m_outMax;
      else if (m_outputSum < m_outMin)
         m_outputSum = m_outMin;

      /* Add Proportional on Error, if P_ON_E is specified */
      double output;
      if (m_pOnE)
         output = m_kp * error;
      else
         output = 0;

      /* Compute Rest of PID Output */
      output += m_outputSum - m_kd * dInput;

      if (output > m_outMax)
         output = m_outMax;
      else if (output < m_outMin)
         output = m_outMin;
      *m_myOutput = output;

      /* Remember some variables for next time */
      m_lastInput = input;
      m_lastTime = now;
      return true;
   }
   else
      return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, int POn) {
   if (Kp < 0 || Ki < 0 || Kd < 0)
      return;

   m_pOn = POn;
   m_pOnE = POn == P_ON_E;

   m_dispKp = Kp;
   m_dispKi = Ki;
   m_dispKd = Kd;

   double SampleTimeInSec = m_sampleTime / 1000.0;
   m_kp = Kp;
   m_ki = Ki * SampleTimeInSec;
   m_kd = Kd / SampleTimeInSec;

   if (m_controllerDirection == REVERSE) {
      m_kp = (0 - m_kp);
      m_ki = (0 - m_ki);
      m_kd = (0 - m_kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd) {
   SetTunings(Kp, Ki, Kd, m_pOn);
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
// why the fuck is this an int ?
void PID::SetSampleTime(int NewSampleTime) {
   if (NewSampleTime > 0) {
      double ratio = (double) NewSampleTime / m_sampleTime;
      m_ki *= ratio;
      m_kd /= ratio;
      m_sampleTime = NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max) {
   if (Min >= Max)
      return;
   m_outMin = Min;
   m_outMax = Max;

   if (m_inAuto) {
      if (*m_myOutput > m_outMax)
         *m_myOutput = m_outMax;
      else if (*m_myOutput < m_outMin)
         *m_myOutput = m_outMin;

      if (m_outputSum > m_outMax)
         m_outputSum = m_outMax;
      else if (m_outputSum < m_outMin)
         m_outputSum = m_outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode) {
   bool newAuto = (Mode == AUTOMATIC);
   if (newAuto && !m_inAuto) {
      // we just went from manual to auto
      PID::Initialize();
   }
   m_inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize() {
   m_outputSum = *m_myOutput;
   m_lastInput = *m_myInput;
   if (m_outputSum > m_outMax)
      m_outputSum = m_outMax;
   else if (m_outputSum < m_outMin)
      m_outputSum = m_outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction) {
   if (m_inAuto && Direction != m_controllerDirection) {
      m_kp = (0 - m_kp);
      m_ki = (0 - m_ki);
      m_kd = (0 - m_kd);
   }
   m_controllerDirection = Direction;
}
