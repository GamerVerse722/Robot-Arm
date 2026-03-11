#include "userapi/ui/driver/debug.hpp"

#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/misc/lv_timer.h"

// #include "gamers-forge/automanager.hpp"
// #include "userapi/configuration.hpp"

static lv_obj_t* labelDebug;

// using namespace AutoManager;

namespace ui::driver::debug {
    void initialize(lv_obj_t* parent) {
        labelDebug = lv_label_create(parent);
        lv_timer_create(debug_timer, 50, nullptr);
    }

    void debug_timer(lv_timer_t* timer) {
        // ez::pose pose = devices::chassis.odom_pose_get();

        // std::string pos_str = std::format("X: {:.2f} Y: {:.2f} Theta: {:.2f}", pose.x, pose.y, pose.theta);

        // std::string autom_mode_str = std::format("Mode: {0}, Color: {1}, Side: {2}", 
        //     Convert::to_string(AutoManager::current_mode),
        //     Convert::to_string(AutoManager::current_color),
        //     Convert::to_string(AutoManager::current_position)
        // );

        // lv_label_set_text(labelDebug, std::format("{}\n{}", pos_str, autom_mode_str).c_str());
    }
}