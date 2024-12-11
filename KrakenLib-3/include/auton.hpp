#pragma once
#include "main.h"

// Function declarations
void autonMogo();
void autonWall();
void autonLift();
void autonIntake();
void trackOdom();
void lateralPIDTune();
void turningPIDTune();
void soloAWP();
void runSelectedAuton();
void autonSelector();
void nothing();
// External variable declarations
extern float xValue;
extern float yValue;
extern float thetaValue;
extern bool autonSelected;
extern int selectedAuton;