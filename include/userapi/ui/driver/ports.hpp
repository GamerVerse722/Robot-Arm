#pragma once

#include "liblvgl/misc/lv_types.h"

#include <string>

namespace ui::driver::ports {
    void initialize(lv_obj_t* parent);
    void create_category(std::string category);
    void create_row(int port_number, std::string port_type, std::string description);
}