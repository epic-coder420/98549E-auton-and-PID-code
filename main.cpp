#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <sys/types.h>

bool toggle = false;

// motor groups
pros::MotorGroup leftmotors({-20, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightmotors({15, 16, 17}, pros::MotorGearset::blue);

// motors
pros::Motor Intake(-3);
pros::Motor Scoring(5);
	

//pneumatics
pros::adi::Pneumatics Piston1('A', false);
pros::adi::Pneumatics Piston2('B', false);
pros::adi::Pneumatics Stopper('C', false);
pros::adi::Pneumatics Descore('D', false);

// Rotation
//IMU
pros::Rotation myrotation(7);
pros::IMU myimu(8);



// Drivetrain
lemlib::Drivetrain drivetrain(&leftmotors, // left motor group
                              &rightmotors, // right motor group
                              13, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// horizontal tracking wheel encoder
pros::Rotation horizontal_tracking(9);

// vertical tracking wheel encoder
pros::Rotation vertical_tracking(10);

//horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking, lemlib::Omniwheel::NEW_325, -5.75);
//vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_tracking, lemlib::Omniwheel::NEW_325, 3.25);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &myimu // inertial sensor
);


// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);


// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();

    // HARD RESET SENSORS
    myimu.reset();
    pros::delay(50);

    vertical_tracking.reset();
    horizontal_tracking.reset();

    // WAIT FOR IMU
    while (myimu.is_calibrating()) {
        pros::lcd::print(0, "IMU CALIBRATING...");
        pros::delay(20);
    }

    pros::lcd::print(0, "IMU READY");

    // NOW calibrate LemLib
    chassis.calibrate();

    pros::lcd::print(1, "LEM CAL DONE");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    // Set starting position
    chassis.setPose(0, 0, 0); // x (in), y (in), heading (deg)

    // Drive forward 24 inches
    chassis.moveToPoint(0, 24, 2000); // x, y, timeout (ms)

    // Turn 90 degrees clockwise
    chassis.turnToHeading(90, 1500);

    // Drive forward another 24 inches
    chassis.moveToPoint(24, 24, 2000);

    // Stop drivetrain
    leftmotors.brake();
    rightmotors.brake();
}




    


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	//pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	//pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	pros::lcd::print(5, "Vert: %f", vertical_tracking.get_position());
	pros::lcd::print(6, "Horiz: %f", horizontal_tracking.get_position());
	// toggle for odom pneumatics
	bool odom_state = false;
	bool prevLeft = false;


	

	// intake toggle state: 0 = off, 1 = X-direction, 2 = Y-direction
	int intake_state = 0;
	bool prevX = false;
	bool prevY = false;

	// scoring toggle state: 0 = off, 1 = B-direction, 2 = A-direction
	int scoring_state = 0;
	bool prevB = false;
	bool prevA = false;

	// piston toggle state: false = retracted, true = deployed
	bool piston_state = false;
	bool prevUp = false;

	// descore toggle state: false = off, true = on
	bool descore_state = false;
	bool prevRight = false;

	//stopper toggle state: false = off, true = on
	bool stopper_state = false;
	bool prevDown = false;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		leftmotors.move(dir + turn);                      // Sets left motor voltage
		rightmotors.move(dir - turn);                     // Sets right motor voltage
		pros::delay(20);  
		
		// Run for 20 ms then update

				// Intake toggle control (press B or A to toggle intake directions) - swapped
		bool curB_btn = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
		bool curA_btn = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);

		// Rising-edge detection for B (now controls intake)
		if (curB_btn && !prevB) {
			if (intake_state == 1) intake_state = 0; // toggle off
			else intake_state = 1; // set B-direction (maps to previous X behavior)
		}

		// Rising-edge detection for A (now controls intake)
		if (curA_btn && !prevA) {
			if (intake_state == 2) intake_state = 0; // toggle off
			else intake_state = 2; // set A-direction (maps to previous Y behavior)
		}

		// Apply motor outputs based on intake_state
		if (intake_state == 1) {
			Intake.move(-127);
			
		} else if (intake_state == 2) {
			Intake.move(127);	
		} else {
			Intake.move(0);
		}

		// save previous button states for intake (B/A)
		prevB = curB_btn;
		prevA = curA_btn;

		// Scoring toggle control (press X or Y to toggle scoring directions) - swapped
		bool curX_btn = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
		bool curY_btn = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

		// Rising-edge detection for X (now controls scoring)
		if (curX_btn && !prevX) {
			if (scoring_state == 1) scoring_state = 0; // toggle off
			else scoring_state = 1; // set X-direction (maps to previous B behavior)
		}

		// Rising-edge detection for Y (now controls scoring)
		if (curY_btn && !prevY) {
			if (scoring_state == 2) scoring_state = 0; // toggle off
			else scoring_state = 2; // set Y-direction (maps to previous A behavior)
		}

		// Apply motor outputs based on scoring_state
		if (scoring_state == 1) {
			Scoring.move(127);
		} else if (scoring_state == 2) {
			Scoring.move(-127);
		} else {
			Scoring.move(0);
		}

		// save previous button states for scoring (X/Y)
		prevX = curX_btn;
		prevY = curY_btn;

		// piston toggle: press UP to toggle both pistons deploy/retract
		bool curUp = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
		if (curUp && !prevUp) {
			piston_state = !piston_state;
			Piston1.set_value(piston_state);
			Piston2.set_value(piston_state);
		}
		prevUp = curUp;

		// Stopper toggle: press DOWN to toggle deploy/retract
		bool curDown = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
		if (curDown && !prevDown) {
			stopper_state = !stopper_state;
			Stopper.set_value(stopper_state);
		}
		prevDown = curDown;

		// Descore toggle: press RIGHT to toggle Descore on/off
		bool curRight = master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
		if (curRight && !prevRight) {
			descore_state = !descore_state;
			Descore.set_value(descore_state);
		}
		prevRight = curRight;

			// give CPU time to other tasks
			pros::delay(10);
		}
}
