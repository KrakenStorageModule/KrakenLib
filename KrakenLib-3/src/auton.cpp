#include "auton.hpp"
#include "devices.hpp"

// Auton Subsystem Functions

// Auton Routines



    //Run No auton
    void nothing(){}
/**
 * \brief A structure containing information about an autonomous routine.
 *
 * This structure contains two fields: a name for the autonomous routine, and
 * a function pointer that implements the autonomous routine.
 *
 * The name field is a null-terminated string that displays the name of the
 * autonomous routine on the controller screen when the user is selecting an
 * autonomous routine to run.
 *
 * The function field is a pointer to a function that implements the autonomous
 * routine. The function should take no arguments, and should not return any
 * value. The function is called by the autonomous selector when the user
 * selects the corresponding autonomous routine.
 */
struct AutonRoutine {
  const char *name;
  void (*function)();
};

// Define the list of auton routines
/// \brief Array of available autonomous routines.
/// 
/// This array holds the list of autonomous routines that can be selected
/// and executed during the autonomous phase. Each routine is represented
/// by a name and a corresponding function pointer that implements the routine.

const AutonRoutine autonRoutines[] = {{"No Auton", nothing}};
const int numAutons = sizeof(autonRoutines) / sizeof(autonRoutines[0]);

int selectedAuton = 0;
bool autonSelected = false;

/**
 * \brief Displays an autonomous selector on the controller screen and allows the user to select an autonomous routine.
 *
 * This function enters an infinite loop where it displays the name of the currently selected autonomous routine on the
 * controller screen, and allows the user to cycle through the available autonomous routines with the left and right
 * buttons. The user can select an autonomous routine by pressing the A button, and can unselect the routine by
 * pressing the X button. The function also provides haptic feedback to the user by rumbling the controller when the
 * user selects or unselects an autonomous routine.
 */
void autonSelector() {
  while (true) {
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
        controller.rumble("."); // Short rumble to confirm selection
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
        controller.rumble("-"); // Different rumble pattern for unselect
      }
    }

    pros::delay(100);
  }
}

// Function to run the selected autonomous routine
/**
 * \brief Runs the autonomous routine selected by the user in the autonomous selector.
 *
 * This function checks if a valid autonomous routine has been selected, and if so, runs it.
 */
void runSelectedAuton() {
  if (selectedAuton >= 0 && selectedAuton < numAutons) {
    autonRoutines[selectedAuton].function();
  }
}