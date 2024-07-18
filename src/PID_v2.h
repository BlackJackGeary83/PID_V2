#ifndef PID_v2_hpp
#define PID_v2_hpp
#define LIBRARY_VERSION 2.0.0

// Constants used in some of the functions below
// Maybe modify these to use `const static` instead for type safety
#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1
#define P_ON_M 0
#define P_ON_E 1

class PID {

public:
    // commonly used functions *************************************

    /*
     * Constructor.  links the PID to the Input, Output, and
     * Setpoint.  Initial tuning parameters are also set here.
     * (overload for specifying proportional mode)
     */
    PID(double* input, double* output, double* setpoint, double Kp, double Ki, double Kd,
        int pOn, int controllerDirection);

    /*
     * Constructor.  links the PID to the Input, Output, and
     * Setpoint.  Initial tuning parameters are also set here.
     */
    PID(double* input, double* output, double* setpoint, double Kp, double Ki, double Kd,
        int controllerDirection);

    // Sets PID to either Manual (0) or Auto (non-0)
    // use bool here ?
    void SetMode(int mode);

    // ez
    bool WrapCompute();

    /*
     * Performs the PID calculation.  it should be
     * called every time loop() cycles. ON/OFF and
     * calculation frequency can be set using SetMode
     * SetSampleTime respectively
     */
    bool Compute();

    /*
     * Clamps the output to a specific range. 0-255 by default, but
     * it's likely the user will want to change this depending on
     * the application
     */
    void SetOutputLimits(double min, double max);

    // available but not commonly used functions ********************

    /*
     * While most users will set the tunings once in the
     * constructor, this function gives the user the option
     * of changing tunings during runtime for Adaptive control
     */
    void SetTunings(double Kp, double Ki, double Kd);

    // overload for specifying proportional mode
    void SetTunings(double Kp, double Ki, double Kd, int pOn);

    /*
     * Sets the Direction, or "Action" of the controller. DIRECT
     * means the output will increase when error is positive. REVERSE
     * means the opposite.  it's very unlikely that this will be needed
     * once it is set in the constructor.
     */
    void SetControllerDirection(int newControllerDirection);

    /*
     * Sets the frequency, in Milliseconds, with which
     * the PID calculation is performed.  default is 100
     */
    void SetSampleTime(int newSampleTime);

    // Display functions ********************************************

    /*
     * These functions query the pid for interal values.
     * they were created mainly for the pid front-end,
     * where it's important to know what is actually
     * inside the PID.
     */
    inline double GetKp() const;
    inline double GetKi() const;
    inline double GetKd() const;
    inline int GetMode() const;
    inline int GetDirection() const;

private:
    void Initialize();

    // we'll hold on to the tuning parameters in user-entered
    // format for display purposes
    double m_dispKp;
    double m_dispKi;
    double m_dispKd;

    double m_kp; // * (P)roportional Tuning Parameter
    double m_ki; // * (I)ntegral Tuning Parameter
    double m_kd; // * (D)erivative Tuning Parameter

    int m_controllerDirection;
    int m_pOn;

    /*
     * Pointers to the Input, Output, and Setpoint variables
     * This creates a hard link between the variables and the
     * PID, freeing the user from having to constantly tell us
     * what these values are.  with pointers we'll just know.
     */
    double* m_myInput;
    double* m_myOutput;
    double* m_mySetpoint;

    unsigned long m_lastTime;
    double m_outputSum, m_lastInput;

    // default Controller Sample Time is 0.1 seconds
    unsigned long m_sampleTime{100};
    // default output limit corresponds to the arduino pwm limits
    double m_outMin{0}, m_outMax{255};
    bool m_inAuto{false}, m_pOnE;
};

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp() const {
    return m_dispKp;
}
double PID::GetKi() const {
    return m_dispKi;
}
double PID::GetKd() const {
    return m_dispKd;
}
int PID::GetMode() const {
    return m_inAuto ? AUTOMATIC : MANUAL;
}
int PID::GetDirection() const {
    return m_controllerDirection;
}

#endif // PID_v2_hpp
