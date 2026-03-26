#pragma once

#include "EZ-Template/PID.hpp"
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
    inline pros::adi::Encoder elbowEncoder('C', 'D', false);
    inline pros::adi::Encoder wristEncoder('E', 'F', false);

    inline ez::PID shoulderPID{0.75, 0, 0.5, 0, "Shoulder"};
    inline ez::PID elbowPID{0.75, 0, 0.5, 0, "Elbow"};
    inline ez::PID wristPID{0.75, 0, 0.5, 0, "Wrist"};

    inline RobotArm arm(
        baseMotor, 
        shoulderMotor, elbowMotor, wristMotor, 
        shoulderEncoder, elbowEncoder, elbowEncoder, 
        shoulderPID, elbowPID, wristPID);
}

namespace configuration::controls {
    extern BMapper::ButtonHandler button_handler;

    void configure();
}