config.h is set up for Bluetooth version, cartesian robot arm and stepper gripper.

For mini servo gripper change line

#define GRIPPER 0 //GRIPPER MOTOR IN USE

to

#define GRIPPER 1 //GRIPPER MOTOR IN USE


For USB version change

#define SERIALX Serial2

to

#define SERIALX Serial


If using the mini servo use a separate power supply to power it.

