#pragma once

#include "liblvgl/misc/lv_types.h"

namespace ui::driver::sponsors {
    void initialize(lv_obj_t* parent);
    void image_render_timer(lv_timer_t *timer);
}