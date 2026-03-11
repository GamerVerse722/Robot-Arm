#pragma once

#include "arm/RobotArm.hpp"
#include "gamers-forge/bmapper.hpp"

#include "pros/misc.hpp"
#include "pros/motors.hpp"


namespace devices {
    extern pros::Controller master;

    extern pros::Motor baseMotor;
    extern pros::Motor shoulderMotor;
    extern pros::Motor elbowMotor;
    extern pros::Motor wristMotor;
    extern pros::Motor clawMotor;

    extern RobotArm arm;
}

namespace configuration::controls {
    extern BMapper::ButtonHandler button_handler;

    void configure();
}