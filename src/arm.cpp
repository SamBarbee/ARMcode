#include "vex.h"
#include "arm.h"

using namespace vex;

double dtr(double deg) {
  return deg*M_PI/180;
}
double rtd(double rad) {
  return rad*(180/M_PI);
}
double locos(double a,double b,double c) {
	return acos((a*a + b*b - c*c) / (2 * a * b));
}
double dist(double x,double y) {
  return sqrt(pow(x,2)+pow(y,2));
}
int sgn(double c) {
  if(c>0)
    return 1;
  if(c<0)
    return -1;
  return 0;
}

RobotArm::RobotArm(motor _mJ1,pot _mJ1_pot, motor _mJ2,pot _mJ2_pot, motor _mJ3,pot _mJ3_pot, motor _mJ4,pot _mJ4_pot, bumper _mEStop)
: mJ1(_mJ1),mJ1_pot(_mJ1_pot), mJ2(_mJ2),mJ2_pot(_mJ2_pot), mJ3(_mJ3), mJ3_pot(_mJ3_pot),mJ4(_mJ4),mJ4_pot(_mJ4_pot),  mEStop(_mEStop) {
  POT_POSITION_T = 0;
}

void RobotArm::eStop() {

  // mJ1.setBrake(hold);
  // mJ2.setBrake(hold);
  // mJ3.setBrake(hold);
  // mJ4.setBrake(hold);

  mJ1.stop();
  mJ2.stop();
  mJ3.stop();
  mJ4.stop();
}

/**
  * Zeros and Calibrates Arm (optional posPosition for "potentiometer center" value)
  */


double RobotArm::getJ1() {
  return (mJ1_pot.value(analogUnits::range12bit)-J1_OFFSET)/(tpd);
}
double RobotArm::getJ2() {
  return -(((mJ2_pot.value(analogUnits::range12bit)-J2_OFFSET)/(tpd*3))-20);
}
double RobotArm::getJ3() {
  return (mJ3_pot.value(analogUnits::range12bit)-J3_OFFSET)/(tpd*3);
}
double RobotArm::getJ4() {
  return (mJ4_pot.value(analogUnits::range12bit)-J4_OFFSET)/(tpd*3);
}

/**
  * Moves the arm to a specific position
  */
void RobotArm::moveToPosition(double x, double y, double z) {
  moveL(x + xOffset, y + yOffset, z + zOffset, 0, 15);
}

void RobotArm::moveToPosition(double x, double y, double z, double a) {
  moveL(x + xOffset, y + yOffset, z + zOffset, a, 15);
}

/**
  * Sets the current arm physical position to the specified positional value
  */
void RobotArm::setPosition(double x, double y, double z) {
  RobotArmFKinematicOutput res;
  forwardKinematicSolve(mJ1.rotation(deg)/7,mJ2.rotation(deg)/3,mJ3.rotation(deg)/3,0.0, res);

  xOffset = res.x - x;
  yOffset = res.y - y;
  zOffset = res.z - z;
}

void RobotArm::setPosition(double x, double y, double z, double a) {
  RobotArmFKinematicOutput res;
  forwardKinematicSolve(mJ1.rotation(deg)/7,mJ2.rotation(deg)/3,mJ3.rotation(deg)/3,0.0, res);

  xOffset = res.x - x;
  yOffset = res.y - y;
  zOffset = res.z - z;
  aOffset = res.a - a;
}

/**
  * Sets the speed of the arm
  */
void RobotArm::setVelocity(double speed, percentUnits units) {
  // TODO: figure this out
}

/**
  * Sets the maximum range of movement for each axis
  */
void RobotArm::setMaxRange(double x, double y, double z) {
  // TODO: figure this out
}
/**
  * Sets the minimum range of movement for each axis
  */
void RobotArm::setMinRange(double x, double y, double z) {
  // TODO: figure this out
}

/**
  * Returns a numeric value for the arm's current position
  */
double RobotArm::getAxisPosition(axisType axis) {
  RobotArmFKinematicOutput res;
  forwardKinematicSolve(mJ1.rotation(deg)/7,mJ2.rotation(deg)/3,mJ3.rotation(deg)/3,0.0, res);
  if (axis == xaxis) {
    return res.x + xOffset;
  } else if (axis == yaxis) {
    return res.y + yOffset;
  } else if (axis == zaxis) {
    return res.z + zOffset;
  }
  return 0;
}

// private functions below

void RobotArm::forwardKinematicSolve(double t, double a1, double b1, double c1) {
  RobotArmFKinematicOutput res;
  forwardKinematicSolve(t, a1, b1, c1, res);
  foutput[0] = res.x;
  foutput[1] = res.y;
  foutput[2] = res.z;
  foutput[3] = res.a;
}

void RobotArm::forwardKinematicSolve(double t, double a1, double b1, double c1, RobotArmFKinematicOutput &output) {
  
  double ch = sqrt(pow(dimensions[2],2)+pow(Z_OFFSET,2));
  double cd = rtd(acos((pow(ch,2)+pow(dimensions[2],2)-pow(Z_OFFSET,2))/(2*ch*dimensions[2])));
  double c2 = 180-cd-(90-c1);
  double tr = ch * sin(dtr(c2));
  double tz = ch * cos(dtr(c2));

  double x1 = dimensions[0] * cos(dtr(90-a1));
  double x2 = dimensions[1] * sin(dtr(-b1));
  
  double z1 = dimensions[0] * sin(dtr(90-a1));
  double z2 = dimensions[1] * cos(dtr(b1));
  
  Brain.Screen.printAt(150,20,"ch:%f",ch);
  Brain.Screen.printAt(150,40,"cd:%f",cd);
  Brain.Screen.printAt(150,60,"c2:%f",c2);
  Brain.Screen.printAt(150,80,"tr:%f",tr);
  Brain.Screen.printAt(150,100,"tz:%f",tz);
  Brain.Screen.printAt(150,120,"x1:%f",x1);
  Brain.Screen.printAt(150,140,"x2:%f",x2);
  Brain.Screen.printAt(150,160,"z1:%f",z1);
  Brain.Screen.printAt(150,180,"z2:%f",z2);

  double r = x1+x2+tr+R_OFFSET;
  output.x = r*cos(dtr(t));
  output.y = r*sin(dtr(t));
  output.z = (z1-z2)-tz;
  output.a = c1;
}

void RobotArm::inverseKinematicSolve(double x,double y,double z, double a) {
  RobotArmIKinematicOutput res;
  inverseKinematicSolve(x, y, z, a, res);
  ioutput[3] = res.j4;
  ioutput[2] = res.j3;
  ioutput[1] = res.j2;
  ioutput[0] = res.j1;
}

void RobotArm::inverseKinematicSolve(double x,double y,double z, double a, RobotArmIKinematicOutput &output) {
  double r = sqrt(pow(x,2)+pow(y,2))-R_OFFSET;

  double ch = sqrt(pow(dimensions[2],2)+pow(Z_OFFSET,2));
  double cd = rtd(acos((pow(ch,2)+pow(dimensions[2],2)-pow(Z_OFFSET,2))/(2*ch*dimensions[2])));
  double c2 = 180-cd-(90-a);
  double tr = ch * sin(dtr(c2));
  double tz = ch * cos(dtr(c2));

  double nr = r - tr;
  double nz = z + tz;
  double rot = atan(y/x);

  Brain.Screen.printAt(280,20,"ch:%f",ch);
  Brain.Screen.printAt(280,40,"cd:%f",cd);
  Brain.Screen.printAt(280,60,"c2:%f",c2);
  Brain.Screen.printAt(280,80,"tr:%f",tr);
  Brain.Screen.printAt(280,100,"tz:%f",tz);
  Brain.Screen.printAt(280,120,"r:%f",r);
  Brain.Screen.printAt(280,140,"nr:%f",nr);
  Brain.Screen.printAt(280,160,"nz:%f",nz);
  Brain.Screen.printAt(280,180,"rot:%f",rot);


  output.j4 = a;
  output.j3=acos((pow(nr,2)+pow(nz,2)-pow(dimensions[0],2)-pow(dimensions[1],2))/(2*(dimensions[0])*(dimensions[1])));
  output.j2=90-(rtd(atan(nz/nr)+atan((dimensions[1]*sin(output.j3))/(dimensions[0]+dimensions[1]*(cos(output.j3))))));
  output.j1=x<0?rtd(rot)+(sgn(y)*180):rtd(rot);
  output.j3= -(180-output.j2-rtd(output.j3));
}

void RobotArm::moveJ(double x, double y, double z, double a, double s){
  RobotArmIKinematicOutput res;
  inverseKinematicSolve(x,y,z,0.0, res);

  mJ1.spinTo(res.j1*7, deg, s, velocityUnits::pct, false);
  mJ2.spinTo(res.j2*3, deg, s, velocityUnits::pct, false);
  mJ3.spinTo(res.j3*3, deg, s, velocityUnits::pct, true);  
}

void RobotArm::moveL(double x, double y, double z, double a, double s){
  RobotArmFKinematicOutput fres;
  forwardKinematicSolve(getJ1(), getJ2(), getJ3(),getJ4(), fres);
  moveLstart[0]=fres.x;
  moveLstart[1]=fres.y;
  moveLstart[2]=fres.z;
  moveLstart[3]=fres.a;
  moveLtarget[0]=x;
  moveLtarget[1]=y;
  moveLtarget[2]=z;
  moveLtarget[3]=a;
  RobotArmIKinematicOutput ires;
  inverseKinematicSolve(x, y, z, a, ires);
  moveLtargetLength = fabs(sqrt(pow(x-fres.x, 2) + pow(y-fres.y, 2) + pow(z-fres.z, 2)));
  calculateMP(0.0, moveLtargetLength, s);  
}

void RobotArm::calculateMP(float startPositionInput, float targetPositionInput, float maxVelocityInput) {

  int NUM_MP_POST_FRAMES = 5;  

  float startPosition = startPositionInput;
  float targetPosition = targetPositionInput;
  float maxVelocity = maxVelocityInput;

// accel duration
  float t1 = 800;
  float t2 = 100;
  float itp = 3;

  float t4 = fabs((targetPosition - startPosition)/maxVelocity) * 1000;

  t4 = (int)(itp * ceil(t4/itp));

  if (t4 < t1 + t2) {
    float total = t1 + t2 + t4;
    float t1t2Ratio = t1/t2;
    int t2Adjusted = floor(total / 2 / (1 + t1t2Ratio) / itp);
    if (t2Adjusted % 2 != 0) {
      t2Adjusted -= 1;
    }
    t2 = t2Adjusted * itp;
    t1 = t2 * t1t2Ratio;
    t4 = total - t1 - t2;
  }

  maxVelocity = fabs((targetPosition - startPosition) / t4) * 1000;

  int numFilter1Boxes = (int)ceil(t1/itp);
  int numFilter2Boxes = (int)ceil(t2/itp);
  int numPoints = (int)ceil(t4/itp);
  int numITP = numPoints + numFilter1Boxes + numFilter2Boxes;
  float filter1 = 0;
  float filter2 = 0;
  float previousVelocity = 0;
  float previousPosition = startPosition;
  float deltaFilter1 = 1.0/numFilter1Boxes;
  float filter2Window[1000];
  int windowIndex = 0;
  int pointIndex = 0;
  if (startPosition > targetPosition && maxVelocity > 0) {
    maxVelocity = -maxVelocity;
  }
  float time = 0;
  float position = 0;
  float velocity = 0;
  float acceleration = 0;

  float next_time = 0;
  float next_position = 0;
  float next_velocity = 0;
  float next_acceleration = 0;

  int lookAhead = 5;

  double x = 0, y = 0, z = 0, a =0;
  double next_x = 0, next_y = 0, next_z = 0, next_a =0;
  double mJ1_error = 0, mJ1_pError = 0;
  double mJ2_error = 0, mJ2_pError = 0;
  double mJ3_error = 0, mJ3_pError = 0;
  double mJ4_error = 0, mJ4_pError = 0;
  double mJ3_integral =0,mJ2_integral=0,mJ1_integral=0,mJ3_tbh=0;
  double mJ1_vel = 0, mJ2_vel = 0, mJ3_vel = 0, mJ4_vel = 0;
  double changeThreshold = 0;
  double mJ1_voltage = 0, mJ2_voltage = 0, mJ3_voltage = 0, mJ4_voltage = 0;

  x=moveLstart[0];
  y=moveLstart[1];
  z=moveLstart[2];
  a=moveLstart[3];
  inverseKinematicSolve(x,y,z,a);

  mJ1_error=ioutput[0]-getJ1();
  mJ2_error=ioutput[1]-getJ2();
  mJ3_error=ioutput[2]-getJ3();
  mJ4_error=ioutput[3]-getJ4();

  //updateARMController(time, position, velocity, acceleration, itp);
  //pointIndex++;
  while (pointIndex <= numITP) {
    int input = (pointIndex - 1) < numPoints ? 1 : 0;
    filter1=input>0?fmin(1, filter1 + deltaFilter1):fmax(0, filter1 - deltaFilter1);
    float firstFilter1InWindow = filter2Window[windowIndex];
    firstFilter1InWindow=pointIndex<=numFilter2Boxes?0:firstFilter1InWindow;
    filter2Window[windowIndex]=filter1;
    filter2+=(filter1-firstFilter1InWindow)/numFilter2Boxes;
    time=pointIndex*itp/1000.0;
    velocity=filter2*maxVelocity;
    position=previousPosition+(velocity+previousVelocity)/2*itp/1000;
    acceleration=(velocity-previousVelocity)/itp*1000;
    previousVelocity=velocity;
    previousPosition=position;
    windowIndex++;
    windowIndex=windowIndex==numFilter2Boxes?0:windowIndex;
    pointIndex++;

    float last_time = time;
    float last_position = position;
    float last_velocity = velocity;
    float last_acceleration = acceleration;

    float burn_filter2=filter2;

    for(int a=1;a<lookAhead;a++) {
      int next_input = ((pointIndex-1)+a) < numPoints ? 1 : 0;
      filter1=next_input>0?fmin(1, filter1 + deltaFilter1):fmax(0, filter1 - deltaFilter1);
      float next_firstFilter1InWindow = filter2Window[windowIndex];
      next_firstFilter1InWindow=pointIndex+a<=numFilter2Boxes?0:next_firstFilter1InWindow;
      // //filter2Window[windowIndex]=filter1;
      burn_filter2+=(filter1-next_firstFilter1InWindow)/numFilter2Boxes;
      next_time=(pointIndex+a)*itp/1000.0;
      next_velocity=burn_filter2*maxVelocity;
      next_position=last_position+(last_velocity+next_velocity)/2*itp/1000;
      next_acceleration=(next_velocity-last_velocity)/itp*1000;

      last_time = next_time;
      last_position = next_position;
      last_velocity = next_velocity;
      last_acceleration = next_acceleration;
    }

    x=(moveLstart[0]+((moveLtarget[0]-moveLstart[0])*position/moveLtargetLength));
    y=(moveLstart[1]+((moveLtarget[1]-moveLstart[1])*position/moveLtargetLength));
    z=(moveLstart[2]+((moveLtarget[2]-moveLstart[2])*position/moveLtargetLength));
    a=(moveLstart[3]+((moveLtarget[3]-moveLstart[3])*position/moveLtargetLength));
    inverseKinematicSolve(x,y,z,a);

    mJ1_error=ioutput[0]-getJ1();
    mJ2_error=ioutput[1]-getJ2();
    mJ3_error=ioutput[2]-getJ3();
    mJ4_error=ioutput[3]-getJ4();

    next_x=(moveLstart[0]+((moveLtarget[0]-moveLstart[0])*next_position/moveLtargetLength));
    next_y=(moveLstart[1]+((moveLtarget[1]-moveLstart[1])*next_position/moveLtargetLength));
    next_z=(moveLstart[2]+((moveLtarget[2]-moveLstart[2])*next_position/moveLtargetLength));
    next_a=(moveLstart[3]+((moveLtarget[3]-moveLstart[3])*next_position/moveLtargetLength));

    inverseKinematicSolve(next_x,next_y,next_z,next_a);
    
    mJ1_vel = (ioutput[0]-(mJ1_error+getJ1()))*(1000/itp);
    mJ2_vel = (ioutput[1]-(mJ2_error+getJ2()))*(1000/itp);
    mJ3_vel = (ioutput[2]-(mJ3_error+getJ3()))*(1000/itp);
    mJ4_vel = (ioutput[3]-(mJ4_error+getJ4()))*(1000/itp);

    mJ1_integral += mJ1_error;
    mJ2_integral += mJ2_error;
    mJ3_integral += mJ3_error;

    // if(sgn(mJ3_error)!=sgn(mJ3_pError)) {
    //   mJ3_integral = 0.5 * (mJ3_integral + mJ3_tbh);
    //   mJ3_tbh = mJ3_integral;
    // }

    mJ1_voltage=mJ1_error*MP_J1_KP+(mJ1_error-mJ1_pError)*MP_J1_KD+mJ1_vel*MP_J1_KV + mJ1_integral*MP_J1_KI + sgn(mJ1_vel)* MP_J1_KF;
    mJ2_voltage=mJ2_error*MP_J2_KP+(mJ2_error-mJ2_pError)*MP_J2_KD+mJ2_vel*MP_J2_KV + mJ2_integral*MP_J2_KI + MP_J2_KF;
    mJ3_voltage=mJ3_error*MP_J3_KP+(mJ3_error-mJ3_pError)*MP_J3_KD+mJ3_vel*MP_J3_KV + mJ3_integral*MP_J3_KI + MP_J3_KF;
    mJ4_voltage=mJ4_error*MP_J4_KP+(mJ4_error-mJ4_pError)*MP_J4_KD+mJ4_vel*MP_J4_KV + sgn(mJ4_vel)*MP_J4_KF;

    mJ1_pError=mJ1_error;
    mJ2_pError=mJ2_error;
    mJ3_pError=mJ3_error;
    mJ4_pError=mJ4_error;

    mJ1.spin(fwd,mJ1_voltage,voltageUnits::volt);
    mJ2.spin(fwd,mJ2_voltage,voltageUnits::volt);
    mJ3.spin(fwd,mJ3_voltage,voltageUnits::volt);
    mJ4.spin(fwd,mJ4_voltage,voltageUnits::volt);

    //dataTester();

    printf("%f,%f,%f,-10,45\n",mJ2_error+getJ2(),getJ2(),mJ2_voltage);

    this_thread::sleep_for(itp);
  }
  for(int i=0;i<NUM_MP_POST_FRAMES;i++) {
    //updateARMController(time, targetPosition, 0, 0, itp);
  }
  mJ1.setBrake(hold);
  mJ2.setBrake(hold);
  mJ3.setBrake(hold);
  mJ1.spin(fwd,0,velocityUnits::pct);
  mJ2.spin(fwd,0,velocityUnits::pct);
  mJ3.spin(fwd,0,velocityUnits::pct);
}

void RobotArm::updateARMController(float time, float position, float velocity, float acceleration, int itp) {
  // double x = 0, y = 0, z = 0, a =0;
  // double mJ1_error = 0, mJ1_pError = 0;
  // double mJ2_error = 0, mJ2_pError = 0;
  // double mJ3_error = 0, mJ3_pError = 0;
  // double mJ4_error = 0, mJ4_pError = 0;
  // double mJ1_vel = 0, mJ2_vel = 0, mJ3_vel = 0, mJ4_vel = 0;
  // double changeThreshold = 0;
  // double mJ1_voltage = 0, mJ2_voltage = 0, mJ3_voltage = 0, mJ4_voltage = 0;

  // x=fabs(moveLtarget[0]-moveLstart[0])>changeThreshold?(moveLstart[0]+((moveLtarget[0]-moveLstart[0])*position/moveLtargetLength)):moveLtarget[0];
  // y=fabs(moveLtarget[1]-moveLstart[1])>changeThreshold?(moveLstart[1]+((moveLtarget[1]-moveLstart[1])*position/moveLtargetLength)):moveLtarget[1];
  // z=fabs(moveLtarget[2]-moveLstart[2])>changeThreshold?(moveLstart[2]+((moveLtarget[2]-moveLstart[2])*position/moveLtargetLength)):moveLtarget[2];
  // a=fabs(moveLtarget[3]-moveLstart[3])>changeThreshold?(moveLstart[3]+((moveLtarget[3]-moveLstart[3])*position/moveLtargetLength)):moveLtarget[3];
  // inverseKinematicSolve(x,y,z,a);
  // mJ1_error=ioutput[0]-getJ1();
  // mJ2_error=ioutput[1]-getJ2();
  // mJ3_error=ioutput[2]-getJ3();
  // mJ4_error=ioutput[3]-getJ4();
  
  // mJ1_vel = mJ1_error*(1000/itp);
  // mJ2_vel = mJ2_error*(1000/itp);
  // mJ3_vel = mJ3_error*(1000/itp);
  // mJ4_vel = mJ4_error*(1000/itp);

  // mJ1_voltage=mJ1_error*MP_J1_KP+(mJ1_error-mJ1_pError)*MP_J1_KD+mJ1_vel*MP_J1_KV;
  // mJ2_voltage=mJ2_error*MP_J2_KP+(mJ2_error-mJ2_pError)*MP_J2_KD+mJ2_vel*MP_J2_KV;
  // mJ3_voltage=mJ3_error*MP_J3_KP+(mJ3_error-mJ3_pError)*MP_J3_KD+mJ3_vel*MP_J3_KV;
  // mJ4_voltage=mJ4_error*MP_J4_KP+(mJ4_error-mJ4_pError)*MP_J4_KD+mJ4_vel*MP_J4_KV;
  // mJ1_pError=mJ1_error;
  // mJ2_pError=mJ2_error;
  // mJ3_pError=mJ3_error;
  // mJ4_pError=mJ4_error;
  // mJ1.spin(fwd,mJ1_voltage,voltageUnits::volt);
  // mJ2.spin(fwd,mJ2_voltage,voltageUnits::volt);
  // mJ3.spin(fwd,mJ3_voltage,voltageUnits::volt);
  // mJ4.spin(fwd,mJ4_voltage,voltageUnits::volt);
  
  // this_thread::sleep_for(1);

  // x=fabs(moveLtarget[0]-moveLstart[0])>changeThreshold?(moveLstart[0]+((moveLtarget[0]-moveLstart[0])*position/moveLtargetLength)):moveLtarget[0];
  // y=fabs(moveLtarget[1]-moveLstart[1])>changeThreshold?(moveLstart[1]+((moveLtarget[1]-moveLstart[1])*position/moveLtargetLength)):moveLtarget[1];
  // z=fabs(moveLtarget[2]-moveLstart[2])>changeThreshold?(moveLstart[2]+((moveLtarget[2]-moveLstart[2])*position/moveLtargetLength)):moveLtarget[2];
  // a=fabs(moveLtarget[3]-moveLstart[3])>changeThreshold?(moveLstart[3]+((moveLtarget[3]-moveLstart[3])*position/moveLtargetLength)):moveLtarget[3];
  // inverseKinematicSolve(x,y,z,a);

  

  // mJ1_error=ioutput[0]-getJ1();
  // mJ2_error=ioutput[1]-getJ2();
  // mJ3_error=ioutput[2]-getJ3();
  // mJ4_error=ioutput[3]-getJ4();
  
  // mJ1_vel += mJ1_error*(1000/itp);
  // mJ2_vel += mJ2_error*(1000/itp);
  // mJ3_vel += mJ3_error*(1000/itp);
  // mJ4_vel += mJ4_error*(1000/itp);

  // mJ1_voltage=mJ1_error*MP_J1_KP+(mJ1_error-mJ1_pError)*MP_J1_KD+mJ1_vel*MP_J1_KV;
  // mJ2_voltage=mJ2_error*MP_J2_KP+(mJ2_error-mJ2_pError)*MP_J2_KD+mJ2_vel*MP_J2_KV;
  // mJ3_voltage=mJ3_error*MP_J3_KP+(mJ3_error-mJ3_pError)*MP_J3_KD+mJ3_vel*MP_J3_KV;
  // mJ4_voltage=mJ4_error*MP_J4_KP+(mJ4_error-mJ4_pError)*MP_J4_KD+mJ4_vel*MP_J4_KV;
  // mJ1_pError=mJ1_error;
  // mJ2_pError=mJ2_error;
  // mJ3_pError=mJ3_error;
  // mJ4_pError=mJ4_error;
  // mJ1.spin(fwd,mJ1_voltage,voltageUnits::volt);
  // mJ2.spin(fwd,mJ2_voltage,voltageUnits::volt);
  // mJ3.spin(fwd,mJ3_voltage,voltageUnits::volt);
  // mJ4.spin(fwd,mJ4_voltage,voltageUnits::volt);

  // printf("%f,%f,%f\n",mJ1_error,mJ1_vel,mJ1_voltage);
  
  // this_thread::sleep_for(1);
  
}

void RobotArm::moveBaseToTargetPosition() {
  double e=0.0, p=0.0, d=0.0, pe=0.0;
  //POT_POSITION_T += POT_OFFSET;
  mJ1.spin(fwd, 0, volt);
  uint32_t endTime = Brain.Timer.system() + 1500; // 1.5 seconds
  while(Brain.Timer.system() < endTime){
    e = POT_POSITION_T - mJ1_Pot.value(range12bit);
    d = pe == 0.0 ? 0.0 : e - pe;
    p = (POT_POSITION_P * e + POT_POSITION_D * d);
    pe = e;
    mJ1.spin(fwd, -p, volt);
    task::sleep(5);
  }
  Brain.Screen.printAt(20,20,"Motion Manager Killed");
}

int RobotArm::mJ1Position(void *arg) {
  if (arg == NULL)
    return(0);
  
  RobotArm &arm = *static_cast<RobotArm *>(arg);

  double e=0.0, p=0.0, d=0.0, pe=0.0;
  //arm.POT_POSITION_T += arm.POT_OFFSET;
  arm.mJ1.spin(fwd, 0, volt);
  while(true){
    e = arm.POT_POSITION_T - arm.mJ1_pot.value(range12bit);
    d = pe == 0.0 ? 0.0 : e - pe;
    p = (arm.POT_POSITION_P * e + arm.POT_POSITION_D * d);
    pe = e;
    arm.mJ1.spin(fwd, -p / 4, volt);
    task::sleep(5);
  }
}

int RobotArm::master(int _J1,int _J2,int _J3,int _J4) {
  if(_J1 > 2200 || _J1 < 1800) {
    Brain.Screen.print("Joint 1 out of range\n");
    return 1;
  }
  else
    J1_OFFSET = _J1;
  if(_J2 > 2130 || _J2 < 2020) {
    Brain.Screen.print("Joint 2 out of range\n");
    return 2;
  }
  else
    J2_OFFSET = _J2;
  if(_J3 > 2350 || _J3 < 2200) {
    Brain.Screen.print("Joint 3 out of range\n");
    return 3;
  }
  else
    J3_OFFSET = _J3;
  if(_J4 > 400 || _J4 < 300) {
    Brain.Screen.print("Joint 4 out of range\n");
    return 4;
  }
  else
    J4_OFFSET = _J4;
  
  return 0;
}

void RobotArm::dataTester() {
  Brain.Screen.printAt(20,20,"1:%f",getJ1());
  Brain.Screen.printAt(20,40,"2:%f",getJ2());
  Brain.Screen.printAt(20,60,"3:%f",getJ3());
  Brain.Screen.printAt(20,80,"4:%f",getJ4());

  forwardKinematicSolve(getJ1(),getJ2(),getJ3(),getJ4());

  Brain.Screen.printAt(20,100,"X:%f",foutput[0]);
  Brain.Screen.printAt(20,120,"Y:%f",foutput[1]);
  Brain.Screen.printAt(20,140,"Z:%f",foutput[2]);
  Brain.Screen.printAt(20,160,"A:%f",foutput[3]);

  inverseKinematicSolve(foutput[0],foutput[1],foutput[2],foutput[3]);

  Brain.Screen.printAt(20,180,"1:%f",ioutput[0]);
  Brain.Screen.printAt(20,200,"2:%f",ioutput[1]);
  Brain.Screen.printAt(20,220,"3:%f",ioutput[2]);
  Brain.Screen.printAt(20,240,"4:%f",ioutput[3]);
}
