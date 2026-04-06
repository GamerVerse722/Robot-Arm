#pragma once

#include "EZ-Template/PID.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

class EncoderMotor {
public:
    EncoderMotor(
        pros::Motor& motor,
        pros::adi::Encoder encoder,
        ez::PID pid
    ):
        motor(motor),
        encoder(encoder),
        pid(pid)
    {}

    void calibrate() {
        motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        motor.tare_position();
        encoder.reset();
    }

    void move(double target, double min = -127.0, double max = 127.0) {
        pid.target_set(target);
        double output = pid.compute(encoder.get_value());

        output = std::clamp(output, min, max);
        motor.move(output);
    }

private:
    pros::Motor& motor;
    pros::adi::Encoder encoder;
    ez::PID pid;
};