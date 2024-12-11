#include "auton.hpp"
#include "devices.hpp"


//Auton Subsystem Functions



//Auton Routines












struct AutonRoutine {
    const char* name;
    void (*function)();
};

// Define the list of auton routines
const AutonRoutine autonRoutines[] = {
    {"Lateral PID Tune", lateralPIDTune},
    {"Turning PID Tune", turningPIDTune},
    {"Solo AWP", soloAWP}
};
const int numAutons = sizeof(autonRoutines) / sizeof(autonRoutines[0]);

int selectedAuton = 0;
bool autonSelected = false;

void autonSelector(){
    while(true){
    if (!autonSelected) {
        // Cycle through autonomous routines with left/right
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            selectedAuton = (selectedAuton + 1) % numAutons;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            selectedAuton = (selectedAuton - 1 + numAutons) % numAutons;
        }
        
        // Select autonomous with A button
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            autonSelected = true;
            controller.rumble(".");  // Short rumble to confirm selection
        }

        // Display selection screen
        controller.set_text(0, 0, autonRoutines[selectedAuton].name);
        controller.set_text(1, 0, "A to Select");
    } else {
        // Show selected autonomous
        controller.set_text(0, 0, autonRoutines[selectedAuton].name);
        controller.set_text(1, 0, "X to Unselect");

        // Unselect with X button
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            autonSelected = false;
            controller.rumble("-");  // Different rumble pattern for unselect
        }
    }

    pros::delay(100);
    }
}

// Function to run the selected autonomous routine
void runSelectedAuton() {
    if (selectedAuton >= 0 && selectedAuton < numAutons) {
        autonRoutines[selectedAuton].function();
    }
}