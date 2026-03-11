#pragma once

#include "liblvgl/core/lv_obj.h"
#include "liblvgl/misc/lv_types.h"

#include "gamers-forge/automanager.hpp"

using namespace AutoManager;

namespace ui::autom::location_selector {
    inline lv_obj_t* location_screen = lv_obj_create(NULL);
    inline AutomMode active_mode = AutomMode::NONE;

    void initialize();

    void register_button(AutomSideColor color, int col, int row);

    void button_event_handler(lv_event_t* e);
}