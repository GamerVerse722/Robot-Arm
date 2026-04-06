#pragma once

#include "EZ-Template/PID.hpp"
#include "gamers-forge/proslogger.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

static PROSLogger::Logger logger{"EncoderMotor"};

class EncoderMotor {
public:
    EncoderMotor(
        pros::Motor& motor,
        pros::adi::Encoder encoder,
        ez::PID pid
    ):
        motor(motor),
        encoder(encoder),
        pid(pid),
        has_negative_pid(false)
    {}

    EncoderMotor(
        pros::Motor& motor,
        pros::adi::Encoder encoder,
        ez::PID pid_pos,
        ez::PID pid_neg
    ):
        motor(motor),
        encoder(encoder),
        pid(pid_pos),
        pid_neg(pid_neg),
        has_negative_pid(true)
    {}

    void calibrate() {
        motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        motor.tare_position();
        encoder.reset();
    }

    void move(double target, double min = -127.0, double max = 127.0) {
        double current = encoder.get_value();
        double error = target - current;

        ez::PID* active_pid = &pid;

        if (has_negative_pid && error > 0) {
            logger.debug("Negative");
            active_pid = &pid_neg;
        }

        active_pid->target_set(target);
        double output = active_pid->compute(current);

        output = std::clamp(output, min, max);
        motor.move(output);
    }

private:
    pros::Motor& motor;
    pros::adi::Encoder encoder;
    ez::PID pid;
    ez::PID pid_neg;
    bool has_negative_pid;
};