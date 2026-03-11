#pragma once
#include "pros/motors.hpp"
#include <cmath>
// #include <algorithm>

class RobotArm {

public:

    RobotArm(pros::Motor& base,
             pros::Motor& shoulder,
             pros::Motor& elbow,
             pros::Motor& wrist)
        : baseMotor(base),
          shoulderMotor(shoulder),
          elbowMotor(elbow),
          wristMotor(wrist)
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

    double X_MIN = 1;
    double X_MAX = 15;

    double Y_MIN = -1;
    double Y_MAX = 15;

    double Z_MIN = -90;
    double Z_MAX = 90;

    /* ---------------- Target Pose ---------------- */

    double targetX = 15;
    double targetY = 0;
    double targetZ = 0;
    double targetWrist = 0;

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
    }

    /* ---------------- Pose update ---------------- */

    void setTarget(double x, double y, double z, double wristDeg) {
        targetX = clamp(x, X_MIN, X_MAX);
        targetY = clamp(y, Y_MIN, Y_MAX);
        targetZ = clamp(z, Z_MIN, Z_MAX);
        targetWrist = wristDeg;
    }

    void adjustTarget(double dx, double dy, double dz) {
        setTarget(targetX + dx,
                  targetY + dy,
                  targetZ + dz,
                  targetWrist);
    }

    void adjustWrist(double d) {
        targetWrist += d;
    }

    /* ---------------- Main update ---------------- */

    void update(int speed = 100) {

        JointAngles j = solveIK(targetX, targetY, targetZ);

        baseMotor.move_absolute(j.base * BASE_RATIO, speed);
        shoulderMotor.move_absolute(j.shoulder * SHOULDER_RATIO, speed);
        elbowMotor.move_absolute(j.elbow * ELBOW_RATIO, speed);
        wristMotor.move_absolute(j.wrist * WRIST_RATIO, speed);
    }

private:

    pros::Motor& baseMotor;
    pros::Motor& shoulderMotor;
    pros::Motor& elbowMotor;
    pros::Motor& wristMotor;

    JointAngles solveIK(double x, double y, double z) {

        JointAngles j;

        /* ---------- Base rotation ---------- */

        j.base = radToDeg(std::atan2(y, x));

        /* ---------- Planar distance ---------- */

        double r = std::sqrt(x*x + y*y);

        double a = ELBOW_LENGTH;
        double b = SHOULDER_LENGTH;

        double rMax = a + b;
        double rMin = std::fabs(a - b);

        /* ---------- Clamp unreachable targets ---------- */

        if (r > rMax) {
            double scale = rMax / r;
            x *= scale;
            y *= scale;
            r = rMax;
        }

        if (r < rMin) {
            double scale = rMin / r;
            x *= scale;
            y *= scale;
            r = rMin;
        }

        double c = r;

        /* ---------- Law of Cosines ---------- */

        double cosA = (b*b + c*c - a*a) / (2*b*c);
        double cosC = (a*a + b*b - c*c) / (2*a*b);

        cosA = clamp(cosA, -1.0, 1.0);
        cosC = clamp(cosC, -1.0, 1.0);

        double A = std::acos(cosA);
        double C = std::acos(cosC);

        /* ---------- Shoulder angle ---------- */

        double targetAngle = std::atan2(z, r);

        j.shoulder = radToDeg(targetAngle - A);

        /* ---------- Elbow angle ---------- */

        j.elbow = radToDeg(M_PI - C);

        /* ---------- Wrist compensation ---------- */

        j.wrist = targetWrist - (j.shoulder + j.elbow);

        return j;
    }

    double clamp(double v, double min, double max) {
        return std::max(min, std::min(v, max));
    }

    double radToDeg(double r) {
        return r * (180.0 / M_PI);
    }
};