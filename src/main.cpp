/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\robert_oakley                                    */
/*    Created:      Thu Jan 09 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// mJ1                  motor         3               
// mJ2                  motor         4               
// mJ3                  motor         2               
// mJ4                  motor         1               
// mStop                bumper        B               
// mJ1_Pot              pot           D               
// ---- END VEXCODE CONFIGURED DEVICES ----


// ---- Robot Arm Controls ----
// All of the robot control info is in robot_arm.cpp 
// This is setup to only run one robot arm to update that to mulitple arms changes will need to be made 
// the following is a list of commands you can use with the robot arm
// e_stop ();            - will stop all motors on robot 
// zero ();              - will zero robot 
// moveL(x,y,z,0,speed); - linear move to called point in linear motion  

#include "vex.h"
#include "stdio.h"
// #include "arm.h"


extern "C" {
  void      vexMagnetPowerSet( uint32_t index, int32_t value, int32_t time );
  int32_t   vexMagnetPowerGet( uint32_t index );
  void      vexMagnetPickup( uint32_t index, V5_DeviceMagnetDuration duration );
  void      vexMagnetDrop( uint32_t index, V5_DeviceMagnetDuration duration );
  double    vexMagnetTemperatureGet( uint32_t index );
  double    vexMagnetCurrentGet( uint32_t index );
  uint32_t  vexMagnetStatusGet( uint32_t index );
  uint32_t  vexMagnetDebugGet( uint32_t index, int32_t id );
  void      vexMagnetModeSet( uint32_t index, uint32_t mode );
  uint32_t  vexMagnetModeGet( uint32_t index );
}

/*

To use the electromagnet:

vexMagnetPickup( PORT3, kMagnetDurationMedium );

*/

using namespace vex;


int robot_loop();

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  if(arm.master(1900,2081,2270,353)!=0) {
    while(true) {
      this_thread::sleep_for(500);
    }
  }

  // e_stop();

  vex::task robot_control(robot_loop);

  while(true){
    if(mStop.pressing()){
      arm.eStop();
      robot_control.stop();
      input_conveyor.stop();
      main_conveyor.stop();
      output_conveyor.stop();
    }
    vex::task::sleep(20);
  }
  
}

int robot_loop(){

  bool TEACHMODE = false; //puts the robot into coast mode and displays positional data on the screen. use this for finding data.

  this_thread::sleep_for(500); //robot will try to kill you if this is removed, and it might succeed. Robot 1 - Sam 0

  while(true){
    
    if(TEACHMODE) {
      Brain.Screen.clearScreen();
      arm.eStop();
      arm.dataTester();
      vex::task::sleep(50);
    }
    else {
      arm.moveToPosition(8,8,-2.5,0);
      arm.moveToPosition(8,-8,-1,90);
    }    

   }

}
