
#ifndef PID_h
#define PID_h
class PID{
  public:
    double kp, ki, kd; // define public variables kp, ki, kd
    // start at 0: PIDs are created fresh inside fwd()/absoluteturn(), and leftover memory in
    // prevError/cumError gave a random first output (fwd() could quit before moving)
    double error = 0, prevError = 0, delta = 0, cumError = 0; // define error, previous error , deltaerror and cumulative error
    double currentTime, previousTime; // timer
    double start = 0; double end = 0; // pausing timestamp
    PID(double , double , double ); // PID inputs
    double getPID(double);
    void pausePID(int);
  

};
#endif
