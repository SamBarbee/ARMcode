#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor mJ1 = motor(PORT2, ratio18_1, false);
motor mJ2 = motor(PORT4, ratio18_1, true);
motor mJ3 = motor(PORT1, ratio18_1, false);
motor mJ4 = motor(PORT3, ratio18_1, false);
bumper mStop = bumper(Brain.ThreeWirePort.H);
pot mJ1_Pot = pot(Brain.ThreeWirePort.C);
pot mJ2_Pot = pot(Brain.ThreeWirePort.A);
pot mJ3_Pot = pot(Brain.ThreeWirePort.D);
pot mJ4_Pot = pot(Brain.ThreeWirePort.B);
motor main_conveyor =motor(PORT1, ratio18_1,false);
motor input_conveyor =motor(PORT6, ratio18_1,false);
motor output_conveyor =motor(PORT9, ratio18_1,false);
motor sorting_arm =motor(PORT10, ratio18_1,false);
line input_track =line(Brain.ThreeWirePort.A);
line output_track =line(Brain.ThreeWirePort.D);
RobotArm arm(mJ1,mJ1_Pot, mJ2,mJ2_Pot, mJ3,mJ3_Pot, mJ4, mJ4_Pot, mStop);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}