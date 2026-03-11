#pragma once

#include "liblvgl/misc/lv_types.h"

#include <string>

namespace ui::driver::keybinds {
    void initialize(lv_obj_t* parent);
    void create_category(std::string category);
    void create_row(std::string key, std::string description);
}