#include "vex.h"

using namespace vex;

// //void conveyor_in(){
//   if(InputTrack.value(analogUnits::range12bit) < 2000)
//   InputBelt.spinFor(forward,1080,deg);
//   MainConveyor.spin(forward,5,pct);
//   InputBelt.stop();
  
  
// }

// void conveyor_out(){
// if(OutputTrack.value(analogUnits::range12bit)< 2600)
// OutputBelt.spinFor(reverse,720,deg);
// }

void sorting_zero(){
  sorting_arm.spin(reverse,10,pct);
  while(sorting_arm.current(amp) < .5){
    vex:task::sleep(20);
  }
  sorting_arm.resetRotation();
  sorting_arm.stop();


}
//oid main_conveyor(){
 // if(InputTrack.value(analogUnits::range12bit) < 2600)
  //MainConveyor.spin(forward,5,pct);
  //else
  //MainConveyor.spin(forward,15,pct);
 
// //}
// void sorting_arm_out(){
//   if(InputTrack.value(analogUnits:: range12bit)< 2000)
//   SortingArm.spinToPosition(110,deg);
//   //SortingArm.stop();
//   else
//   SortingArm.spinToPosition(0,deg);

//   }


  


