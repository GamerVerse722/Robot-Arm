#include "userapi/ui/driver/debug.hpp"

#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/misc/lv_timer.h"
#include "userapi/arm/RobotArm.hpp"
#include "userapi/configuration.hpp"

static lv_obj_t* labelDebug;

namespace ui::driver::debug {
    void initialize(lv_obj_t* parent) {
        labelDebug = lv_label_create(parent);
        lv_timer_create(debug_timer, 50, nullptr);
    }

    void debug_timer(lv_timer_t* timer) {
        using namespace devices;
        std::string target_pos = std::format("X: {:.2f}, Y: {:.2f}, Z: {:.2f}, Wrist: {:.2f}",
            arm.targetX,
            arm.targetY,
            arm.targetZ,
            arm.targetWrist);

        std::string motor_angles = std::format("Sholder: {:.2f}, Elbow: {:.2f}, Wrist: {:.2f}",
            shoulderMotor.get_position(),
            elbowMotor.get_position(),
            wristMotor.get_position());

        std::string motor_angles_scaled = std::format("Sholder: {:.2f}, Elbow: {:.2f}, Wrist: {:.2f}",
            shoulderMotor.get_position() * (12.0/60.0),
            elbowMotor.get_position() * (12.0/60.0),
            wristMotor.get_position() * (12.0/60.0));

        std::string controller_speed = std::format("LX: {:.2f}, LY: {:.2f}, RX: {:.2f}",
            master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0 * 10.0,
            master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0 * 10.0,
            master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0 * 10.0
        );
        
        lv_label_set_text(labelDebug, std::format("{}\n{}\n{}\n{}",
            target_pos,
            motor_angles,
            motor_angles_scaled,
            controller_speed
        ).c_str());
    }
}