#pragma once

#include "pros/motors.h"
#include "pros/motors.hpp"
#include "userapi/arm/EncoderMotor.hpp"

#include <algorithm>
#include <cmath>

namespace arm::config {
    /* ---------------- Geometry ---------------- */

    const double SHOULDER_LENGTH = 7.0;
    const double ELBOW_LENGTH = 8.5;

    /* ---------------- Gear ratios ---------------- */

    const double BASE_RATIO = 84.0 / 36.0;
    const double SHOULDER_RATIO = 60.0 / 12.0;
    const double ELBOW_RATIO = 60.0 / 12.0;
    const double WRIST_RATIO = 60.0 / 12.0;

    /* ---------------- Workspace limits ---------------- */

    const double X_MIN = 5.0;
    const double X_MAX = 15.5;

    const double Y_MIN = -1.0;
    const double Y_MAX = 15.5;

    const double Z_MIN = -90.0;
    const double Z_MAX = 90.0;

    const double WRIST_MIN = -90.0;
    const double WRIST_MAX = 90.0;
}

namespace arm {
    class RobotArm {
    public:
        RobotArm(
            pros::Motor& base,
            EncoderMotor& shoulderMotor,
            EncoderMotor& elbowMotor,
            EncoderMotor& wristMotor
        ):
            baseMotor(base),
            shoulderMotor(shoulderMotor),
            elbowMotor(elbowMotor),
            wristMotor(wristMotor)
        {}

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

        void calibrate() {
            baseMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            baseMotor.tare_position();

            shoulderMotor.calibrate();
            elbowMotor.calibrate();
            wristMotor.calibrate();
        }

        /* ---------------- Pose update ---------------- */

        void setTarget(double x, double y, double z, double wristDeg) {
            using namespace arm::config;

            double distance = std::sqrt(x*x + y*y);
            const double maxReach = SHOULDER_LENGTH + ELBOW_LENGTH - 0.1;
            const double minReach = std::abs(SHOULDER_LENGTH - ELBOW_LENGTH);

            if (distance < 1e-6) distance = 1e-6;

            double scale = 1.0;

            if (distance > maxReach) {
                scale = maxReach / distance;
            } else if (distance < minReach) {
                scale = minReach / distance;
            }

            x *= scale;
            y *= scale;

            targetX = std::clamp(x, X_MIN, X_MAX);
            targetY = std::clamp(y, Y_MIN, Y_MAX);

            targetZ = std::clamp(z, Z_MIN, Z_MAX);
            targetWrist = wristDeg;
        }

        void adjustTarget(const double dx, const double dy) {
            using namespace arm::config;

            setTarget(
                targetX + dx,
                targetY + dy,
                targetZ,
                targetWrist
            );
        }

        void adjustWrist(const double d) {
            using namespace arm::config;

            setTarget(
                targetX,
                targetY,
                targetZ,
                targetWrist + d
            );
        }

        void adjustBase(const double dz) {
            using namespace arm::config;
            
            setTarget(
                targetX,
                targetY,
                targetZ + dz,
                targetWrist
            );
        }

        /* ---------------- Main update ---------------- */
        void update() {
            using namespace arm::config;

            JointAngles j = solveIK(targetX, targetY, targetZ, targetWrist);

            // Base (still motor encoder)
            // baseMotor.move_absolute(j.base * BASE_RATIO, 127);

            // --- SHOULDER (Okapi PID + ADI encoder) ---
            shoulderMotor.move(j.shoulder * SHOULDER_RATIO);
            elbowMotor.move(j.elbow * ELBOW_RATIO);
            wristMotor.move(j.wrist * WRIST_RATIO);
        }

        JointAngles solveIK(const double x, const double y, const double z, const double wrist) {
            using namespace arm::config;

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

        EncoderMotor& shoulderMotor;
        EncoderMotor& elbowMotor;
        EncoderMotor& wristMotor;

        double radToDeg(const double r) {
            return r * (180.0 / M_PI);
        }
    };
}