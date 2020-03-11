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

  //arm.calibrate(-1800);
  //arm.moveToPosition(.6, -9, 1.5);
      //Move robot arm to central location
  vex::task::sleep(50);

  

  // sorting_arm.spin(reverse,10,pct);
  // while(sorting_arm.current(amp) < 0.5){
  //   vex:task::sleep(20);
  // }
  // sorting_arm.resetRotation();
  // sorting_arm.stop();




  vex::task::sleep(250);

  //arm.eStop();

  while(true){
    
    Brain.Screen.clearScreen();
    arm.eStop();
    arm.dataTester();
    vex::task::sleep(50);

    //arm.moveToPosition(5.63,5.01,0,0);

    // arm.moveToPosition(6,-2,-1.1,0);
    // arm.moveToPosition(6,2,-1.1,0);

    //arm.moveToPosition(5.63,-5.01,0,0);



    // arm.moveToPosition(4,7,-1,0);
    // arm.moveToPosition(4,7,-3,0);
    // arm.moveToPosition(4,7,-1,0);
    // arm.moveToPosition(4,-5,-1,0);
    // arm.moveToPosition(4,-5,-4.0,90);
    // arm.moveToPosition(7,-8,-4.0,90);
    // arm.moveToPosition(4,-5,-4.0,90);
    
    // arm.moveToPosition(4,-5,-2.5,0);

  //    while(output_track.value(analogUnits::range12bit)< 2800){
  //   //If the output line tracker sees a puck
  //     vex::task::sleep(50);

  //     while(output_track.value(analogUnits::range12bit) > 1500) {
  //       vex::task::sleep(50);
  //     }
 
  //     output_conveyor.stop();
  //     vex::task::sleep(50);
       // arm.moveToPosition(6,1,-1);
  //      arm.moveToPosition(-9,-5.5,.15);
  //      vexMagnetPickup( PORT3, kMagnetDurationLong );
  //     arm.moveToPosition(-9,-5.5,.9);
  //     sorting_arm.spinToPosition(0,degrees,false);
  //     arm.moveToPosition(8.5,-5.2,.9);
  //      arm.moveToPosition(8.5,-5.2,0.25);
  //      vexMagnetDrop( PORT3, kMagnetDurationMedium );
  //      arm.moveToPosition(8.5,-5.2,.9);
  //      arm.moveToPosition(-9.0,-5.4,.9);

  // }
    
  //    while(input_track.value(analogUnits::range12bit) < 2800){
  //   // //When the input line tracker sees a puck then
  //    input_conveyor.setVelocity(85,percent);
  //   // //Set input belt speed to 100 percent
  //    input_conveyor.spinFor(forward,1500,degrees);
  //   // //Rotate input belt 1500 degrees to enter puck onto conveyor belt
  //    sorting_arm.spinToPosition(90,degrees); 
  //    output_conveyor.spin(reverse,40,pct);
  //   // //Rotate sorting arm to 100 degrees for puck output
  //    input_conveyor.stop();
     
  //    }
  //   // //Stop the input belt

  //    if(sorting_arm.position(deg) > 10){
  //   //   output_conveyor.spin(reverse);
  //    //If the sorting arm is at the output location, run the output belt motor
  //    }
  //    if(input_track.value(analogUnits::range12bit) > 2800) {
  //    vex::task::sleep(250);
  //    main_conveyor.spin(forward,15,pct);
  //   //If the input line tracker doesn't see a puck, spin the main conveyor at 15 percent
  //   }

   }



   
  


}
