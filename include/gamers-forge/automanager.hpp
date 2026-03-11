#pragma once

#include <functional>
#include <string_view>

namespace AutoManager {
    enum AutomMode {
        QUALIFICATIONS,
        ELIMINATIONS,
        AWP,
        SKILLS,
        NONE
    };
    
    enum AutomPosition {
        LEFT,
        RIGHT,
        NO_POSITION
    };

    enum AutomColor {
        RED,
        BLUE,
        COLOR_NONE
    };
    
    enum AutomSideColor {
        RED_LEFT,
        RED_RIGHT,
        BLUE_LEFT,
        BLUE_RIGHT,
        NO_COLOR_AND_POSITION,
    };

    using callback_method = std::function<void()>;

    extern AutomMode current_mode;
    extern AutomSideColor current_side;
    extern AutomPosition current_position;
    extern AutomColor current_color;
    extern callback_method current_autom;

    void register_autom(AutomMode mode, AutomColor color, AutomPosition position, callback_method callback);

    void select_autom(AutomMode mode, AutomSideColor side_color);

    void run_autom();
}

namespace AutoManager::Convert {
    using namespace AutoManager;
    
    AutomSideColor to_side_color(AutomColor color, AutomPosition pos);
    AutomColor     side_color_to_color(AutomSideColor sc);
    AutomPosition  side_color_to_position(AutomSideColor sc);

    std::string_view to_string(AutomMode);
    std::string_view to_string(AutomColor);
    std::string_view to_string(AutomPosition);
    std::string_view to_string(AutomSideColor);
}