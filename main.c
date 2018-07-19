#pragma config(Sensor, dgtl1,  rTracker,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  lTracker,       sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  hTracker,       sensorQuadEncoder)
#pragma config(Motor,  port2,           driveL1,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port3,           driveL2,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           driveR1,       tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           driveR2,       tmotorVex393TurboSpeed_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

float xPos; //var holding robot's x coordinate , relative to startup position , updates every 5ms
float yPos; //var holding robot's y coordinate , relative to startup position , updates every 5ms
float theta; //var holding robot's heading , relative to startup position , updates every 5 ms

#include "miscFunctions.h"
#include "PIDstruct.h"
#include "PID.h"
#include "posControl.h"

task main()
{

	xPos = yPos = theta = 0; //initializes coordinates and heading to 0

	startTask(trackPos); //posTracking task

	moveTo(30,40); //test positions

}
