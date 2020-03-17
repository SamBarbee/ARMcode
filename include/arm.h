#ifndef VEX_ROBOT_ARM_h
#define VEX_ROBOT_ARM_h

using namespace vex;

struct RobotArmFKinematicOutput {
  double x;
  double y;
  double z;
  double a;
};

struct RobotArmIKinematicOutput {
  double j1;
  double j2;
  double j3;
  double j4;
};

class RobotArm {
private:

  double xOffset=0, yOffset=0, zOffset=0,aOffset=0;

  int BaseMotor_Pot_Position();

  double dimensions[3] = {7.0,7.0,4.0};
  double R_OFFSET=0.722;
  double Z_OFFSET=5.0;
  
  double J1_OFFSET;
  double J2_OFFSET;
  double J3_OFFSET;
  double J4_OFFSET;

  double pot_res = 4096; //resolution of the pot
  double pot_rot = 333; //pot of measurement
  double tpd = pot_res/pot_rot;
  double dpt = pot_rot/pot_res;

  int POT_POSITION_T = 0;
  double POT_POSITION_P = 0.075;
  double POT_POSITION_D = POT_POSITION_P*4.0;

  double foutput[4];
  double ioutput[4];
  double moveLstart[4];
  double moveLtarget[4];
  double moveLtargetLength;

  double MP_J1_KP = 0.125;
  double MP_J1_KI = 0.0005;
  double MP_J1_KD = MP_J1_KP*4.0;
  double MP_J1_KV = 0.01;//0.00325;
  double MP_J1_KA = 0.0;
  double MP_J1_KF = 0.0;
  bool MP_J1_INVERT_A=false;

  double MP_J2_KP = 0.75;
  double MP_J2_KI = 0.001;
  double MP_J2_KD = MP_J2_KP*4;
  double MP_J2_KV = 0.0125;
  double MP_J2_KA = 0.0;
  double MP_J2_KF = 0;
  bool MP_J2_INVERT_A=false;

  double MP_J3_KP = MP_J2_KP;
  double MP_J3_KI = MP_J2_KI;
  double MP_J3_KD = MP_J3_KP*4;
  double MP_J3_KV = MP_J2_KV;
  double MP_J3_KA = 0.0;
  double MP_J3_KF = 1.0;
  bool MP_J3_INVERT_A=false;

  double MP_J4_KP = MP_J2_KP;
  double MP_J4_KD = MP_J4_KP*4.0;
  double MP_J4_KV = MP_J2_KV;
  double MP_J4_KF = 0.0;
  double MP_J4_KA = 0.0;

  void forwardKinematicSolve(double t, double a1, double b1, double c1);
  void forwardKinematicSolve(double t, double a1, double b1, double c1, RobotArmFKinematicOutput &output);
  void inverseKinematicSolve(double x,double y,double z, double a);
  void inverseKinematicSolve(double x,double y,double z,double a, RobotArmIKinematicOutput &output);

  void moveJ(double x, double y, double z, double a, double s);
  void moveL(double x, double y, double z, double a, double s);
  void calculateMP(float startPositionInput, float targetPositionInput, float maxVelocityInput);
  void updateARMController(float time, float position, float velocity, float acceleration, int itp);
  void moveBaseToTargetPosition();
  static int mJ1Position(void *arg);

  double getJ1();
  double getJ2();
  double getJ3();
  double getJ4();

public:
  motor mJ1;
  motor mJ2;
  motor mJ3;
  motor mJ4;
  pot mJ1_pot;
  pot mJ2_pot;
  pot mJ3_pot;
  pot mJ4_pot;
  bumper mEStop;

  RobotArm(motor _mJ1,pot _mJ1_pot, motor _mJ2,pot _mJ2_pot, motor _mJ3,pot _mJ3_pot, motor _mJ4,pot _mJ4_pot, bumper _mEStop);

  void eStop();
  
  /**
   * Zeros and Calibrates Arm (optional posPosition for "potentiometer center" value)
   */
  //void calibrate(int potPosition=0);

  int master(int _J1,int _J2,int _J3,int _J4);

  /**
   * Moves the arm to a specific position
   */
  void moveToPosition(double x, double y, double z);

  void moveToPosition(double x, double y, double z, double a);

  /**
   * Sets the current arm physical position to the specified positional value
   */
  void setPosition(double x, double y, double z);
  void setPosition(double x, double y, double z, double a);

  /**
   * Sets the speed of the arm
   */
  void setVelocity(double speed, percentUnits units=percent);

  /**
   * Sets the maximum range of movement for each axis
   */
  void setMaxRange(double x, double y, double z);

  /**
   * Sets the minimum range of movement for each axis
   */
  void setMinRange(double x, double y, double z);

  /**
   * Returns a numeric value for the arm's current position
   */
  double getAxisPosition(axisType axis);

  void dataTester();

};

#endif