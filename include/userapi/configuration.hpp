#pragma once

#include "EZ-Template/PID.hpp"
#include "arm/RobotArm.hpp"
#include "gamers-forge/bmapper.hpp"

#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "userapi/arm/EncoderMotor.hpp"

namespace devices {
    inline pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    inline pros::Motor baseMotor     (6,  pros::MotorGearset::red);
    inline pros::Motor shoulderMotor (-5, pros::MotorGearset::red);
    inline pros::Motor elbowMotor    (4,  pros::MotorGearset::green);
    inline pros::Motor wristMotor    (3,  pros::MotorGearset::green);
    inline pros::Motor clawMotor     (2,  pros::MotorGearset::green);
    
    inline pros::adi::Encoder shoulderEncoder ('A', 'B', false);
    inline pros::adi::Encoder elbowEncoder    ('C', 'D', true);
    inline pros::adi::Encoder wristEncoder    ('E', 'F', true);

    inline ez::PID shoulderPID      {0.5, 0, 1, 0, "Shoulder"};
    inline ez::PID elbowPID         {0.5, 0, 1, 0, "Elbow"};
    inline ez::PID elbowNegativePID {0.3, 0, 2, 0, "ElbowNegative"};
    inline ez::PID wristPID         {1.0, 0, 0.75, 0, "Wrist"};

    inline EncoderMotor shoulderEM {shoulderMotor, shoulderEncoder,     shoulderPID};
    inline EncoderMotor elbowEM    {elbowMotor,    elbowEncoder,    elbowPID, elbowNegativePID};
    inline EncoderMotor wristEM    {wristMotor,    wristEncoder,        wristPID};

    inline arm::RobotArm robotArm(
        baseMotor,
        shoulderEM,
        elbowEM,
        wristEM
    );
}

namespace configuration::controls {
    extern BMapper::ButtonHandler button_handler;

    void configure();
}