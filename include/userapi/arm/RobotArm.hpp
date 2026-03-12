#pragma once
#include "pros/motors.hpp"
#include <algorithm>
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

    double WRIST_MIN = -90;
    double WRIST_MAX = 90;

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
        targetX = std::clamp(x, X_MIN, X_MAX);
        targetY = std::clamp(y, Y_MIN, Y_MAX);
        targetZ = std::clamp(z, Z_MIN, Z_MAX);
        targetWrist = std::clamp(wristDeg, WRIST_MIN, WRIST_MAX);
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

        j.base = z;

        /* ---------- Planar distance ---------- */

        double a = ELBOW_LENGTH;
        double b = SHOULDER_LENGTH;
        double c = std::sqrt(x*x + y*y);


        double rMax = a + b;
        double rMin = std::fabs(a - b);

        /* ---------- Clamp unreachable targets ---------- */

        if (c > rMax) {
            double scale = rMax / c;
            x *= scale;
            y *= scale;
            c = rMax;
        }

        if (c < rMin) {
            double scale = rMin / c;
            x *= scale;
            y *= scale;
            c = rMin;
        }

        /* ---------- Law of Cosines ---------- */

        double cosA = (b*b + c*c - a*a) / (2*b*c);
        double cosB = (c*c + a*a - b*b) / (2*a*c);
        double cosC = (a*a + b*b - c*c) / (2*a*b);

        cosA = std::clamp(cosA, -1.0, 1.0);
        cosB = std::clamp(cosB, -1.0, 1.0);
        cosC = std::clamp(cosC, -1.0, 1.0);

        double A = std::acos(cosA);
        double B = std::acos(cosB);
        double C = std::acos(cosC);

        /* ---------- Shoulder angle ---------- */

        double targetAngle = std::atan2(x, y);

        j.shoulder = radToDeg(targetAngle + A);

        /* ---------- Elbow angle ---------- */

        j.elbow = radToDeg(C);

        /* ---------- Wrist compensation ---------- */

        // Need to find correct algorithm
        j.wrist = targetWrist - (j.shoulder + j.elbow);

        return j;
    }

    double radToDeg(double r) {
        return r * (180.0 / M_PI);
    }
};