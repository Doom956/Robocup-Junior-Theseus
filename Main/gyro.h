#ifndef gyro_h
#define gyro_h
// timer class
class gyro{
  public:
    
    gyro();
    void init_Gyro();
    double heading();
    double pitch_heading();
    int inverse(int,bool);
    int modulus(int);
    int headingToCardinal(double);
    void reset_accel_filter();
    double opposite_heading(double);
    // Squared up against a wall: take the drift out of the heading (see gyro.cpp).
    bool resyncToNearestCardinal(double maxCorrectionDeg);
    private:
      bool accelFilterInitialized = false;
      double headingOffset = 0; // subtracted from the BNO055 heading; set by resyncToNearestCardinal()
      double accelFiltered = 0.0;
      double v;
      unsigned long lastTime;

};
#endif