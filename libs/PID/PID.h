/* PID.h-Simple PID Library
   Original Library-Arduino-PID-Library
   Edited by H.P
*/

#ifndef PID_h
#define PID_h
#include "Arduino.h"

class PID
{


  public:
  //contrctr and main fn s
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and
        double, double, double);     //   Setpoint.  Initial tuning parameters are also set here

    bool Compute();                       // * performs the PID calculation. Returns True if
                                          // there's a change and gives the required output


    void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
										  //editable

    void Initialize();                    //Set up the controller


  //other fn s
    void SetTunings(double, double,       // foradaptive contrl in runtime
                    double);

    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which
                                          //   the PID calculation is performed.  default is 100

  private:

	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter


    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;
    double *mySetpoint;
	unsigned long lastTime;
	double ITerm, lastInput;

	unsigned long SampleTime;
	double outMin, outMax;

};
#endif
