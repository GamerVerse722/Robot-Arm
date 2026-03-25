#pragma once

#include "arm/RobotArm.hpp"
#include "gamers-forge/bmapper.hpp"

#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"

namespace devices {
    inline pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    inline pros::Motor baseMotor(6, pros::MotorGearset::red);
    inline pros::Motor shoulderMotor(-5, pros::MotorGearset::red);
    inline pros::Motor elbowMotor(4, pros::MotorGearset::green);
    inline pros::Motor wristMotor(3, pros::MotorGearset::green);
    inline pros::Motor clawMotor(2, pros::MotorGearset::green);

    inline pros::adi::Encoder shoulderEncoder('A', 'B', false);

    inline RobotArm arm(baseMotor, shoulderMotor, elbowMotor, wristMotor, shoulderEncoder);
}

namespace configuration::controls {
    extern BMapper::ButtonHandler button_handler;

    void configure();
}