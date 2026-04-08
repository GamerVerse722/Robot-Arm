#include "main.h"

#include "gamers-forge/proslogger.hpp"

#include "liblvgl/display/lv_display.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"

#include "userapi/configuration.hpp"
#include "userapi/ui/autom/mode_selector.hpp"
#include "userapi/ui/op_control.hpp"

double driveCurveExpo(double input, double deadzone, double gain) {
    if (std::abs(input) < deadzone) return 0;

    double sign = input > 0 ? 1 : -1;
    double x = (std::abs(input) - deadzone) / (127 - deadzone);

    // Exponential curve
    double curved = (std::exp(gain * x) - 1) / (std::exp(gain) - 1);

    return sign * curved * 127;
}


using namespace devices;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	PROSLogger::Manager::setLevel(PROSLogger::LogLevel::DEBUG);

	pros::delay(500);

	ui::driver::initialize();
	
	configuration::controls::configure();

	// lv_screen_load(ui::autom::mode_selector::mode_screen);
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
void competition_initialize() {
	lv_screen_load(ui::autom::mode_selector::mode_screen);
	// lv_screen_load(ui::driver::driver_screen);
}

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
	using namespace ui::autom;
	// lv_screen_load(ui::driver::driver_screen);
	AutoManager::run_autom();
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
	lv_screen_load(ui::driver::driver_screen);
	configuration::controls::button_handler.start();

    double speed = 0.25 / 2.0;
	double deadzone = 20;

    while (true) {

        double lx = devices::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        double ly = devices::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        double rx = devices::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		double ry = -devices::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);


		lx = driveCurveExpo(lx, deadzone, 1.2) / 127.0;
		ly = driveCurveExpo(ly, deadzone, 1.2) / 127.0;
		rx = driveCurveExpo(rx, deadzone, 1.2) / 127.0;
		ry = driveCurveExpo(ry, deadzone, 1.2) / 127.0;

        robotArm.adjustTarget(
            lx * (speed),
            ly * speed
        );

		devices::baseMotor.move(rx * 25.0);
		// robotArm.adjustBase(rx * 1.0);
		robotArm.adjustWrist(ry);

        robotArm.update();

        pros::delay(10);
    }
}