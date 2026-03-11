#pragma once

#include "liblvgl/misc/lv_types.h"

namespace ui::driver::debug {
    void initialize(lv_obj_t* parent);
    void debug_timer(lv_timer_t* timer);
}