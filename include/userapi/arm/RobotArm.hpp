#pragma once

#include "EZ-Template/PID.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

#include <algorithm>
#include <cmath>

class RobotArm {

public:

    RobotArm(
        pros::Motor& base,
        pros::Motor& shoulder,
        pros::Motor& elbow,
        pros::Motor& wrist,
        pros::adi::Encoder& encoder,
        ez::PID& shoulderPID
    ):
        baseMotor(base),
        shoulderMotor(shoulder),
        elbowMotor(elbow),
        wristMotor(wrist),
        shoulderEncoder(encoder),
        shoulderPID(shoulderPID)
    {
        baseMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        shoulderMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        elbowMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        wristMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    /* ---------------- Geometry ---------------- */

    const double SHOULDER_LENGTH = 7.0;
    const double ELBOW_LENGTH = 8.5;

    /* ---------------- Gear ratios ---------------- */

    const double BASE_RATIO = 84.0 / 36.0;
    const double SHOULDER_RATIO = 60.0 / 12.0;
    const double ELBOW_RATIO = 60.0 / 12.0;
    const double WRIST_RATIO = 60.0 / 12.0;

    /* ---------------- Workspace limits ---------------- */

    double X_MIN = 3.0;
    double X_MAX = 15.5;

    double Y_MIN = -1.0;
    double Y_MAX = 15.5;

    double Z_MIN = -90.0;
    double Z_MAX = 90.0;

    double WRIST_MIN = -90.0;
    double WRIST_MAX = 90.0;

    /* ---------------- Arm Speed ---------------- */

    const int ARM_SPEED = 75;
    const int BASE_SPEED = 75;

    /* ---------------- Target Pose ---------------- */

    double targetX = 15.5;
    double targetY = 0.0;
    double targetZ = 0.0;
    double targetWrist = 0.0;

    struct JointAngles {
        double base;
        double shoulder;
        double elbow;
        double wrist;
    };

    void calibrateVertical() {
        baseMotor.tare_position();
        shoulderMotor.tare_position();
        elbowMotor.tare_position();
        wristMotor.tare_position();
        shoulderEncoder.reset();
    }

    /* ---------------- Pose update ---------------- */

    void setTarget(double x, double y, double z, double wristDeg) {
        targetX = std::clamp(x, X_MIN, X_MAX);
        targetY = std::clamp(y, Y_MIN, Y_MAX);
        targetZ = std::clamp(z, Z_MIN, Z_MAX);
        targetWrist = wristDeg;
        // targetWrist = std::clamp(wristDeg, WRIST_MIN, WRIST_MAX);
    }

    void adjustTarget(double dx, double dy) {
        setTarget(
            targetX + dx,
            targetY + dy,
            targetZ,
            targetWrist
        );
    }

    void adjustWrist(double d) {
        setTarget(
            targetX,
            targetY,
            targetZ,
            targetWrist + d
        );
    }

    void adjustBase(double dz) {
        setTarget(
            targetX,
            targetY,
            targetZ + dz,
            targetWrist
        );
    }

    /* ---------------- Main update ---------------- */
    void update() {

        JointAngles j = solveIK(targetX, targetY, targetZ, targetWrist);

        // Base (still motor encoder)
        baseMotor.move_absolute(j.base * BASE_RATIO, BASE_SPEED);

        // --- SHOULDER (Okapi PID + ADI encoder) ---
        motorEncoder(j.shoulder * SHOULDER_RATIO, shoulderMotor, shoulderEncoder, shoulderPID);

        // Other joints (still motor-based)
        elbowMotor.move_absolute(j.elbow * ELBOW_RATIO, ARM_SPEED);
        wristMotor.move_absolute(j.wrist * WRIST_RATIO, ARM_SPEED);
    }

    void motorEncoder(double target, pros::Motor& motor, pros::adi::Encoder& encoder, ez::PID pid) {
        pid.target_set(target);
        double output = pid.compute(encoder.get_value());

        output = std::clamp(output, -127.0, 127.0);
        motor.move(output);
    }

    JointAngles solveIK(const double x, const double y, const double z, const double wrist) {
        JointAngles joint_angles{};

        // Join Base
        joint_angles.base = z;

        // Side Length
        const double a = ELBOW_LENGTH;
        const double b = SHOULDER_LENGTH;
        const double c = std::sqrt(x*x + y*y);

        // Cosine Law
        const double A = std::acos(std::clamp(((b*b + c*c - a*a) / (2*b*c)), -1.0, 1.0));
        const double B = std::acos(std::clamp(((c*c + a*a - b*b) / (2*a*c)), -1.0, 1.0));
        const double C = std::acos(std::clamp(((a*a + b*b - c*c) / (2*a*b)), -1.0, 1.0));

        // Shoulder Angle
        const double targetAngle = std::atan2(y,x);
        joint_angles.shoulder = radToDeg(targetAngle + A);

        // Elbow Angle
        joint_angles.elbow = radToDeg(M_PI - C);

        // Wrist Angle
        const double shoulderX = b * cos(targetAngle + A);
        const double shoulderY = b * sin(targetAngle + A);

        const double wristAngle = -std::atan2(y-shoulderY, x-shoulderX);
        joint_angles.wrist = radToDeg(wristAngle) - wrist;

        return joint_angles;
    }

private:
    pros::Motor& baseMotor;
    pros::Motor& shoulderMotor;
    pros::Motor& elbowMotor;
    pros::Motor& wristMotor;

    pros::adi::Encoder& shoulderEncoder;

    ez::PID& shoulderPID;

    const double TICKS_PER_REV = 360.0;

    double radToDeg(double r) {
        return r * (180.0 / M_PI);
    }

    double getShoulderAngle() {
        return (shoulderEncoder.get_value() / TICKS_PER_REV) * 360.0;
    }
};