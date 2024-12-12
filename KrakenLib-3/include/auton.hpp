#pragma once
// Function declarations
#include <string>
void trackOdom();
void runSelectedAuton();
void autonSelector();
void nothing();
void red();
void blue();
void test();
// External variable declarations
extern float xValue;
extern float yValue;
extern float thetaValue;
extern bool autonSelected;
extern int selectedAuton;
extern std::string color;