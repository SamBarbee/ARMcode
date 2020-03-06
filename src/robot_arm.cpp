#include "vex.h"

// using namespace vex;

// void updateARMController(float time, float position, float velocity, float acceleration, int itp);
// void calculateMP(float startPositionInput, float targetPositionInput, float maxVelocityInput);

// void forwardKinematicSolve(double t, double a1, double b1);
// void inverseKinematicSolve(double x,double y,double z);

// double dtr(double deg);
// double rtd(double rad);
// double locos(double a,double b,double c);
// double dist(double x,double y);
// int sgn(double c);

// int mJ1_Pot_Position();

// double dimensions[2] = {7.0,7.0};
// double R_OFFSET=2.5625;
// double POT_OFFSET=2560;

// double J1_TpD = 5.6;
// double J2_TpD = 5.6;
// double J3_TpD = 5.6;

// int POT_POSITION_T = 0;
// double POT_POSITION_P = 0.075;
// double POT_POSITION_D = POT_POSITION_P*4.0;

// double foutput[3];
// double ioutput[3];
// double moveLstart[3];
// double moveLtarget[3];
// double moveLtargetLength;

// double MP_J1_KP = 0.25;//1.5;
// double MP_J1_KD = MP_J1_KP*4.0;
// double MP_J1_KV = 0.075;
// double MP_J1_KA = 0.0;
// double MP_J1_KF = 1.5;
// bool MP_J1_INVERT_A=false;

// double MP_J2_KP = 0.9;
// double MP_J2_KD = MP_J2_KP*4.0;
// double MP_J2_KV = 0.075;
// double MP_J2_KA = 0.0;
// double MP_J2_KF = 0.0;
// bool MP_J2_INVERT_A=false;

// double MP_J3_KP = MP_J2_KP;
// double MP_J3_KD = MP_J3_KP*4.0;
// double MP_J3_KV = MP_J2_KV;
// double MP_J3_KA = 0.0;
// bool MP_J3_INVERT_A=false;

// void e_stop(){

//     mJ1.stop();
//     mJ2.stop();
//     mJ3.stop();
//     mJ4.stop();
// }

// void zero() {


//     POT_POSITION_T=-800;

//     // mJ2.spin(fwd, -20, pct);
//     // mJ2.setBrake(hold);
//     // this_thread::sleep_for(400);
//     // mJ2.spin(fwd,0,pct);

//     vex::task mJ1_Motion_Manager(mJ1_Pot_Position);



//     this_thread::sleep_for(1000);

//     mJ1.setRotation(0.0,deg);

//     mJ1_Motion_Manager.stop();
    

//     // while(mJ1_Pot.value(analogUnits::range12bit) >= 2510 ){   // This the the potentiometer value you can change for homing 
//     //   mJ1.spin(fwd,10,pct);
//     // }
//     // mJ1.spin(fwd, 0, pct);
//     // this_thread::sleep_for(250);
//     // mJ1.spinTo(0,deg,25,velocityUnits::pct,true); // This can change start after homing 

//     mJ4.spin(fwd, -10, pct);
//     while(mJ4.current(amp)<0.75){
//       this_thread::sleep_for(10);
//     }
//     mJ4.stop();
//     this_thread::sleep_for(500);
//     mJ4.setRotation(0.0, deg);
//     mJ3.spin(fwd, -10, pct);
//     while(mJ3.current(amp)<0.625){
//       this_thread::sleep_for(10);
//     }
//     mJ3.spin(fwd, 0, pct);
//     this_thread::sleep_for(250);
//     mJ3.setRotation(-48.0*3, deg);
//     mJ3.spinTo(0, deg);
//     mJ2.setBrake(coast);
//     this_thread::sleep_for(1000);
//     mJ2.setBrake(coast);
//     mJ2.spin(fwd, 20, pct);
//     this_thread::sleep_for(150);
//     while(mJ2.current(amp)<0.625){
//       this_thread::sleep_for(10);
//     }
//     mJ2.spin(fwd, 0, pct);
//     mJ2.setRotation(180, deg);
//     this_thread::sleep_for(250);
//     mJ2.spinTo(150, deg);
//     this_thread::sleep_for(250);
    
//     mJ1.spinTo(0,deg,25,velocityUnits::pct,false);
//     mJ2.spinTo(30*3,deg,25,velocityUnits::pct,false);
//     mJ3.spinTo(-120,deg,25,velocityUnits::pct,false);
//     mJ4.spinTo(0,deg,25,velocityUnits::pct,false);
//     this_thread::sleep_for(1000);
// }

// void forwardKinematicSolve(double t, double a1, double b1) {
//   double a2 = 90.0-b1;
//   double b2 = 90.0-a1;
//   double x1 = (dimensions[0] * sin(dtr(a1)))/sin(dtr(90));
//   double x2 = (dimensions[1] * sin(dtr(b1)))/sin(dtr(90));
//   double z1 = (dimensions[0] * sin(dtr(b2)))/sin(dtr(90));
//   double z2 = (dimensions[1] * sin(dtr(a2)))/sin(dtr(90));
//   double r = (x1-x2)+R_OFFSET;
//   foutput[0] = r*cos(dtr(t));
//   foutput[1] = r*sin(dtr(t));
//   foutput[2] = z1-z2;
// }

// void inverseKinematicSolve(double x,double y,double z) {
//     double r = sqrt(pow(x,2)+pow(y,2))-R_OFFSET;
//     double a = atan(y/x);
//     ioutput[2]=acos((pow(r,2)+pow(z,2)-pow(dimensions[0],2)-pow(dimensions[1],2))/(2*(dimensions[0])*(dimensions[1])));
//     ioutput[1]=90-(rtd(atan(z/r)+atan((dimensions[1]*sin(ioutput[2]))/(dimensions[0]+dimensions[1]*(cos(ioutput[2]))))));
//     ioutput[0]=rtd(a);
//     ioutput[2]= -(180-ioutput[1]-rtd(ioutput[2]));
// }

// void moveJ(double x, double y, double z, double a, double s){
//   inverseKinematicSolve(x,y,z);

//   mJ1.spinTo(ioutput[0]*7,deg,s,velocityUnits::pct,false);
//   mJ2.spinTo(ioutput[1]*3,deg,s,velocityUnits::pct,false);
//   mJ3.spinTo(ioutput[2]*3,deg,s,velocityUnits::pct,true);  
// }

// void moveL(double x, double y, double z, double a, double s){
//   forwardKinematicSolve(mJ1.rotation(deg)/7,mJ2.rotation(deg)/3,mJ3.rotation(deg)/3);
//   moveLstart[0]=foutput[0];
//   moveLstart[1]=foutput[1];
//   moveLstart[2]=foutput[2];
//   moveLtarget[0]=x;
//   moveLtarget[1]=y;
//   moveLtarget[2]=z;
//   inverseKinematicSolve(x,y,z);
//   moveLtargetLength = fabs(sqrt(pow(x-foutput[0],2)+pow(y-foutput[1],2)+pow(z-foutput[2],2)));
//   MP_J1_INVERT_A = (mJ1.rotation(deg)/7) > ioutput[0] ? true : false;
//   MP_J2_INVERT_A = (mJ2.rotation(deg)/3) > ioutput[1] ? true : false;
//   MP_J3_INVERT_A = (mJ3.rotation(deg)/3) > ioutput[2] ? true : false;
//   calculateMP(0.0,moveLtargetLength,s);  
// }


// double dtr(double deg) {
//   return deg*M_PI/180;
// }
// double rtd(double rad) {
//   return rad*(180/M_PI);
// }
// double locos(double a,double b,double c) {
// 	return acos((a*a + b*b - c*c) / (2 * a * b));
// }
// double dist(double x,double y) {
//   return sqrt(pow(x,2)+pow(y,2));
// }
// int sgn(double c) {
//   if(c>0)
//     return 1;
//   if(c<0)
//     return -1;
//   return 0;
// }

// void calculateMP(float startPositionInput, float targetPositionInput, float maxVelocityInput) {

//   int NUM_MP_POST_FRAMES = 5;  

//   float startPosition = startPositionInput;
// 	float targetPosition = targetPositionInput;
// 	float maxVelocity = maxVelocityInput;

// 	float t1 = 100;
// 	float t2 = 100;
// 	float itp = 10;

// 	float t4 = fabs((targetPosition - startPosition)/maxVelocity) * 1000;

// 	t4 = (int)(itp * ceil(t4/itp));

// 	if (t4 < t1 + t2) {
// 		float total = t1 + t2 + t4;
// 		float t1t2Ratio = t1/t2;
// 		int t2Adjusted = floor(total / 2 / (1 + t1t2Ratio) / itp);
// 		if (t2Adjusted % 2 != 0) {
// 			t2Adjusted -= 1;
// 		}
// 		t2 = t2Adjusted * itp;
// 		t1 = t2 * t1t2Ratio;
// 		t4 = total - t1 - t2;
// 	}

// 	maxVelocity = fabs((targetPosition - startPosition) / t4) * 1000;

// 	int numFilter1Boxes = (int)ceil(t1/itp);
// 	int numFilter2Boxes = (int)ceil(t2/itp);
// 	int numPoints = (int)ceil(t4/itp);
// 	int numITP = numPoints + numFilter1Boxes + numFilter2Boxes;
// 	float filter1 = 0;
// 	float filter2 = 0;
// 	float previousVelocity = 0;
// 	float previousPosition = startPosition;
// 	float deltaFilter1 = 1.0/numFilter1Boxes;
// 	float filter2Window[20];
// 	int windowIndex = 0;
// 	int pointIndex = 0;
// 	if (startPosition > targetPosition && maxVelocity > 0) {
// 		maxVelocity = -maxVelocity;
// 	}
// 	float time = 0;
// 	float position = 0;
// 	float velocity = 0;
// 	float acceleration = 0;
// 	updateARMController(time, position, velocity, acceleration, itp);
// 	pointIndex++;
// 	while (pointIndex <= numITP) {
// 		int input = (pointIndex - 1) < numPoints ? 1 : 0;
//     filter1=input>0?fmin(1, filter1 + deltaFilter1):fmax(0, filter1 - deltaFilter1);
// 		float firstFilter1InWindow = filter2Window[windowIndex];
//     firstFilter1InWindow=pointIndex<=numFilter2Boxes?0:firstFilter1InWindow;
// 		filter2Window[windowIndex]=filter1;
// 		filter2+=(filter1-firstFilter1InWindow)/numFilter2Boxes;
// 		time=pointIndex*itp/1000.0;
// 		velocity=filter2*maxVelocity;
// 		position=previousPosition+(velocity+previousVelocity)/2*itp/1000;
// 		acceleration=(velocity-previousVelocity)/itp*1000;
// 		previousVelocity=velocity;
// 		previousPosition=position;
// 		windowIndex++;
//     windowIndex=windowIndex==numFilter2Boxes?0:windowIndex;
// 		updateARMController(time,position,velocity,acceleration,itp);
// 		pointIndex++;
// 	}
// 	for(int i=0;i<NUM_MP_POST_FRAMES;i++) {
// 		updateARMController(time, targetPosition, 0, 0, itp);
// 	}
//   mJ1.setBrake(hold);
//   mJ2.setBrake(hold);
//   mJ3.setBrake(hold);
//   mJ1.spin(fwd,0,velocityUnits::pct);
//   mJ2.spin(fwd,0,velocityUnits::pct);
//   mJ3.spin(fwd,0,velocityUnits::pct);
// }
// void updateARMController(float time, float position, float velocity, float acceleration, int itp) {
//   double x,y,z,mJ1_error,mJ1_pError,mJ2_error,mJ2_pError,mJ3_error,mJ3_pError,mJ1_vel,mJ2_vel,mJ3_vel,changeThreshold = 0.2,mJ1_voltage,mJ2_voltage,mJ3_voltage;

//   x=fabs(moveLtarget[0]-moveLstart[0])>changeThreshold?(moveLstart[0]+((moveLtarget[0]-moveLstart[0])*position/moveLtargetLength)):moveLtarget[0];
//   y=fabs(moveLtarget[1]-moveLstart[1])>changeThreshold?(moveLstart[1]+((moveLtarget[1]-moveLstart[1])*position/moveLtargetLength)):moveLtarget[1];
//   z=fabs(moveLtarget[2]-moveLstart[2])>changeThreshold?(moveLstart[2]+((moveLtarget[2]-moveLstart[2])*position/moveLtargetLength)):moveLtarget[2];
//   inverseKinematicSolve(x,y,z);
//   mJ1_error=ioutput[0]-mJ1.rotation(deg)/7;
//   mJ2_error=ioutput[1]-mJ2.rotation(deg)/3;
//   mJ3_error=ioutput[2]-mJ3.rotation(deg)/3;
  
//   mJ1_vel = mJ1_error*(100/itp);
//   mJ2_vel = mJ2_error*(100/itp);
//   mJ3_vel = mJ3_error*(100/itp);

//   x=fabs(moveLtarget[0]-moveLstart[0])>changeThreshold?(moveLstart[0]+((moveLtarget[0]-moveLstart[0])*position/moveLtargetLength)):moveLtarget[0];
//   y=fabs(moveLtarget[1]-moveLstart[1])>changeThreshold?(moveLstart[1]+((moveLtarget[1]-moveLstart[1])*position/moveLtargetLength)):moveLtarget[1];
//   z=fabs(moveLtarget[2]-moveLstart[2])>changeThreshold?(moveLstart[2]+((moveLtarget[2]-moveLstart[2])*position/moveLtargetLength)):moveLtarget[2];
//   inverseKinematicSolve(x,y,z);
//   mJ1_error=ioutput[0]-mJ1.rotation(deg)/7;
//   mJ2_error=ioutput[1]-mJ2.rotation(deg)/3;
//   mJ3_error=ioutput[2]-mJ3.rotation(deg)/3;

//     // mJ1_accel = 
//     // mJ2_accel
//     // mJ3_accel


//     mJ1_voltage=mJ1_error*MP_J1_KP+(mJ1_error-mJ1_pError)*MP_J1_KD+mJ1_vel*MP_J1_KV;//+acceleration*MP_J1_KA;
//     mJ2_voltage=mJ2_error*MP_J2_KP+(mJ2_error-mJ2_pError)*MP_J2_KD+mJ2_vel*MP_J2_KV;//+acceleration*MP_J2_KA;
//     mJ3_voltage=mJ3_error*MP_J3_KP+(mJ3_error-mJ3_pError)*MP_J3_KD+mJ3_vel*MP_J3_KV;//+acceleration*MP_J3_KA;
//     mJ1_pError=mJ1_error;
//     mJ2_pError=mJ2_error;
//     mJ3_pError=mJ3_error;
//     mJ1.spin(fwd,mJ1_voltage,voltageUnits::volt);
//     mJ2.spin(fwd,mJ2_voltage,voltageUnits::volt);
//     mJ3.spin(fwd,mJ3_voltage,voltageUnits::volt);

//     Brain.Screen.printAt(20,20,"1:%f",mJ1.rotation(rotationUnits::deg));
//     Brain.Screen.printAt(20,40,"2:%f",mJ2.rotation(rotationUnits::deg));
//     Brain.Screen.printAt(20,60,"3:%f",mJ3.rotation(rotationUnits::deg));
//     Brain.Screen.printAt(20,80,"3:%f",mJ3.rotation(rotationUnits::raw));
    
//     this_thread::sleep_for(1);

//   //for(int j=0;j<(int)itp;j++){
//     x=fabs(moveLtarget[0]-moveLstart[0])>changeThreshold?(moveLstart[0]+((moveLtarget[0]-moveLstart[0])*position/moveLtargetLength)):moveLtarget[0];
//     y=fabs(moveLtarget[1]-moveLstart[1])>changeThreshold?(moveLstart[1]+((moveLtarget[1]-moveLstart[1])*position/moveLtargetLength)):moveLtarget[1];
//     z=fabs(moveLtarget[2]-moveLstart[2])>changeThreshold?(moveLstart[2]+((moveLtarget[2]-moveLstart[2])*position/moveLtargetLength)):moveLtarget[2];
//     inverseKinematicSolve(x,y,z);
//     mJ1_error=ioutput[0]-mJ1.rotation(deg)/7;
//     mJ2_error=ioutput[1]-mJ2.rotation(deg)/3;
//     mJ3_error=ioutput[2]-mJ3.rotation(deg)/3;

//     // mJ1_accel = 
//     // mJ2_accel
//     // mJ3_accel


//     mJ1_voltage=mJ1_error*MP_J1_KP+(mJ1_error-mJ1_pError)*MP_J1_KD+mJ1_vel*MP_J1_KV;//+acceleration*MP_J1_KA;
//     mJ2_voltage=mJ2_error*MP_J2_KP+(mJ2_error-mJ2_pError)*MP_J2_KD+mJ2_vel*MP_J2_KV;//+acceleration*MP_J2_KA;
//     mJ3_voltage=mJ3_error*MP_J3_KP+(mJ3_error-mJ3_pError)*MP_J3_KD+mJ3_vel*MP_J3_KV;//+acceleration*MP_J3_KA;
//     mJ1_pError=mJ1_error;
//     mJ2_pError=mJ2_error;
//     mJ3_pError=mJ3_error;
//     mJ1.spin(fwd,mJ1_voltage+(MP_J1_KF*sgn(mJ1_error)),voltageUnits::volt);
//     mJ2.spin(fwd,mJ2_voltage,voltageUnits::volt);
//     mJ3.spin(fwd,mJ3_voltage,voltageUnits::volt);

//     Brain.Screen.printAt(20,20,"1:%f",mJ1.rotation(rotationUnits::deg));
//     Brain.Screen.printAt(20,40,"2:%f",mJ2.rotation(rotationUnits::deg));
//     Brain.Screen.printAt(20,60,"3:%f",mJ3.rotation(rotationUnits::deg));
//     Brain.Screen.printAt(20,80,"3:%f",mJ3.rotation(rotationUnits::raw));
    
//     this_thread::sleep_for(9);
//   //}
  
// }

// void print_xyz(){
//   Brain.Screen.clearScreen();
//   forwardKinematicSolve(mJ1.rotation(deg)/7,mJ2.rotation(deg)/3,mJ3.rotation(deg)/3);
//   Brain.Screen.printAt(20,20,"X:%f",foutput[0]);
//   Brain.Screen.printAt(20,40,"Y:%f",foutput[1]);
//   Brain.Screen.printAt(20,60,"Z:%f",foutput[2]);
  
//   vex::task::sleep(100);
// }

// int mJ1_Pot_Position() {
//   double e=0.0,p=0.0,d=0.0,pe=0.0;
//   POT_POSITION_T+=POT_OFFSET;
//   mJ1.spin(fwd,0,voltageUnits::volt);
//   while(true){
//     // e=POT_POSITION_T-mJ1_Pot.value(analogUnits::range12bit);
//     d = pe == 0.0 ? 0.0 : e - pe;
//     p=(POT_POSITION_P*e + POT_POSITION_D*d);
//     pe=e;
//     //mJ1.spin(fwd,-p,velocityUnits::pct);
//     mJ1.spin(fwd,-p,voltageUnits::volt);
//     vex::task::sleep(5);
//   }
// }
