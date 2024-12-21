#include "auton.hpp"
#include "KrakenLib/2dmp.hpp"
#include "KrakenLib/interface.hpp"
#include "KrakenLib/odom.hpp"
#include "KrakenLib/point.hpp"
#include "KrakenLib/ramsete.hpp"
#include "KrakenLib/splineGen.hpp"
#include "devices.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"


using std::string;
// Declare odom object here
Odometry odom(imu, &parallelTracker, &perpedicularTracker, 3.25, .75, 2.75, 0,
              0);

// Declare RAMSETE Controller
RamseteController ramseteController(1, 2, 3, 4, 76.76);
// Declare Spline Generation Tool
SplineGen splineGen;
// Declare Motion Profile
MotionProfile MotionProfile(76.76, 12.0);
// Auton Subsystem Functions
void autonIntake(string state) {
  if (state == "intake") {
    intakeHook.move_voltage(1200000);
    intakeFront.move_voltage(1200000);
  } else if (state == "outtake") {
    intakeHook.move_voltage(-1200000);
    intakeFront.move_voltage(-1200000);
  } else if (state == "kill") {
    intakeHook.move_voltage(0);
    intakeFront.move_voltage(0);
  }
}
// torque value needs to be adjusted

/**
 * @brief Runs an anti-jam routine for the intake.
 *
 * If the intake's torque is above 1.1, this function will
 * run a routine to "unjam" the intake. This is done by
 * briefly reversing the intake's voltage, then returning
 * the voltage to its original setting.
 *
 * This function is meant to be run in a separate task.
 */
void antijam() {
  double attemptedVoltage = intakeHook.get_voltage();
  while (true) {
    if (intakeHook.get_torque() > 1.1) {
      intakeHook.move_voltage(-120000);
      pros::c::delay(200);
      intakeHook.move_voltage(attemptedVoltage);
    }
  }
}
// Displays Position on Brain Screen
/**
 * @brief Updates Odom and displays position on Brain Screen -> Intended to be
 * run as a task
 */
void trackOdom() {
  while (true) {
    odom.update();
    pros::lcd::print(0, "x: %f", odom.getX());
    pros::lcd::print(1, "y: %f", odom.getY());
    pros::lcd::print(2, "theta: %f", odom.getHeading());
    pros::delay(10);
  }
}
// Auton Routines

// Run No auton
void nothing() { pros::Task trackOdomTask(trackOdom); }

void test() {
  pros::Task antijamTask(antijam);
  pros::Task trackOdomTask(trackOdom);
  // adding points
  Point point1 = Point(0, 0, 0, 0, 0);  // start point
  Point point2 = Point(0, 10, 0, 0, 0); // end point
  MotionProfile.addWaypoint(point1);    // adding start point to path
  MotionProfile.addWaypoint(point2);    // adding end point to path
  // running path
  run(ramseteController, odom, MotionProfile, splineGen);
  autonIntake("intake");                // Starts the intake until it is killed
  Point point3 = Point(0, 0, 0, 0, 0);  // start point
  Point point4 = Point(0, 10, 0, 0, 0); // end point
  MotionProfile.addWaypoint(point3);    // adding start point to path
  MotionProfile.addWaypoint(point4);    // adding end point to path
  // running path
  run(ramseteController, odom, MotionProfile, splineGen);
  autonIntake("kill"); // Stops the intake after movement is finished
}
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

const AutonRoutine autonRoutines[] = {{"No Auton", nothing}, {"Test", test}};
const int numAutons = sizeof(autonRoutines) / sizeof(autonRoutines[0]);

int selectedAuton = 0;
bool autonSelected = false;

/**
 * \brief Displays an autonomous selector on the controller screen and allows
 * the user to select an autonomous routine.
 *
 * This function enters an infinite loop where it displays the name of the
 * currently selected autonomous routine on the controller screen, and allows
 * the user to cycle through the available autonomous routines with the left and
 * right buttons. The user can select an autonomous routine by pressing the A
 * button, and can unselect the routine by pressing the X button. The function
 * also provides haptic feedback to the user by rumbling the controller when the
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
 * \brief Runs the autonomous routine selected by the user in the autonomous
 * selector.
 *
 * This function checks if a valid autonomous routine has been selected, and if
 * so, runs it.
 */
void runSelectedAuton() {
  if (selectedAuton >= 0 && selectedAuton < numAutons) {
    autonRoutines[selectedAuton].function();
  }
}