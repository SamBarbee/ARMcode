using namespace vex;

#include "arm.h"

extern brain Brain;

// VEXcode devices
extern motor mJ1;
extern motor mJ2;
extern motor mJ3;
extern motor mJ4;
extern bumper mStop;
extern pot mJ1_Pot;
extern line input_track;
extern line output_track;
extern motor main_conveyor;
extern motor output_conveyor;
extern motor input_conveyor;
extern RobotArm arm;
extern motor sorting_arm;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );