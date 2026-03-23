#include "userapi/configuration.hpp"

#include "pros/misc.h"

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include "userapi/arm/RobotArm.hpp"

namespace devices {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    pros::Motor baseMotor(6);
    pros::Motor shoulderMotor(-5);
    pros::Motor elbowMotor(4);
    pros::Motor wristMotor(3);
    pros::Motor clawMotor(2);

    RobotArm arm(baseMotor, shoulderMotor, elbowMotor, wristMotor);
}

namespace configuration::controls {
    BMapper::ButtonHandler button_handler(devices::master);

    void configure() {
        using pros::controller_digital_e_t;

        button_handler.bind(pros::E_CONTROLLER_DIGITAL_R1)
            .onHold([] { devices::arm.adjustWrist(1.5); })
            .apply();

        button_handler.bind(pros::E_CONTROLLER_DIGITAL_R2)
            .onHold([] { devices::arm.adjustWrist(-1.5); })
            .apply();

        button_handler.bind(pros::E_CONTROLLER_DIGITAL_L1)
            .onHold([] { devices::clawMotor.move(100); })
            .onRelease([] { devices::clawMotor.move(0); })
            .apply();

        button_handler.bind(pros::E_CONTROLLER_DIGITAL_L2)
            .onHold([] { devices::clawMotor.move(-100); })
            .onRelease([] { devices::clawMotor.move(0); })
            .apply();
    }
} 