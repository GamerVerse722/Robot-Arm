#include "userapi/configuration.hpp"

#include "pros/misc.h"

#include "pros/motors.h"
#include "pros/motors.hpp"

#include "userapi/arm/RobotArm.hpp"

namespace configuration::controls {
    BMapper::ButtonHandler button_handler(devices::master);

    void configure() {
        using pros::controller_digital_e_t;

        button_handler.bind(pros::E_CONTROLLER_DIGITAL_R1)
            .onPress([] { devices::robotArm.adjustWrist(1.0); })
            .onHold([] { devices::robotArm.adjustWrist(1.0); })
            .apply();

        button_handler.bind(pros::E_CONTROLLER_DIGITAL_R2)
            .onPress([] { devices::robotArm.adjustWrist(-1.0); })
            .onHold([] { devices::robotArm.adjustWrist(-1.0); })
            .apply();

        button_handler.bind(pros::E_CONTROLLER_DIGITAL_L1)
            .onHold([] { devices::clawMotor.move(100); })
            .onRelease([] { devices::clawMotor.move(0); })
            .apply();

        button_handler.bind(pros::E_CONTROLLER_DIGITAL_L2)
            .onHold([] { devices::clawMotor.move(-100); })
            .onRelease([] { devices::clawMotor.move(0); })
            .apply();

        devices::clawMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
} 