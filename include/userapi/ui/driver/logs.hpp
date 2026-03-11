#pragma once

#include "liblvgl/misc/lv_types.h"

#include "gamers-forge/proslogger.hpp"

namespace ui::driver::logs {
    void initialize(lv_obj_t* parent);
    void logs(PROSLogger::LoggerEvent event);
}