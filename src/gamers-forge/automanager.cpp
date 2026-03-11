#include "gamers-forge/automanager.hpp"

#include <cstddef>
#include <format>
#include <string_view>

#include "gamers-forge/proslogger.hpp"

namespace AutoManager {
    static PROSLogger::Logger log{"Auto Manager"};

    // Fixed size callback arrays
    static std::array<callback_method, 5> qualification_methods {};
    static std::array<callback_method, 5> elimination_methods {};
    static std::array<callback_method, 5> awp_methods {};
    static callback_method skills_callback {nullptr};
    static callback_method none_callback {nullptr};

    // Current Selection
    AutomMode current_mode = AutomMode::NONE;
    AutomSideColor current_side = AutomSideColor::NO_COLOR_AND_POSITION;
    AutomPosition current_position = AutomPosition::NO_POSITION;
    AutomColor current_color = AutomColor::COLOR_NONE;
    callback_method current_autom = nullptr;

    void register_autom(AutomMode mode, AutomColor color, AutomPosition position, callback_method callback) {
        AutomSideColor side_color = Convert::to_side_color(color, position);
        size_t index = static_cast<size_t>(side_color);

        switch (mode) {
            case AutomMode::QUALIFICATIONS:
                if (qualification_methods[index]) { log.warn("Overwriting existing QUALIFICATION callback"); }
                else { log.debug("Registering QUALIFICATION callback"); }

                qualification_methods[index] = callback;
                break;

            case AutomMode::ELIMINATIONS:
                if (elimination_methods[index]) { log.warn("Overwriting existing ELIMINATION callback"); }
                else { log.debug("Registering ELIMINATION callback"); }

                elimination_methods[index] = callback;
                break;

            case AutomMode::AWP:
                if (awp_methods[index]) { log.warn("Overwriting existing AWP callback"); }
                else { log.debug("Registering AWP callback"); }

                awp_methods[index] = callback;
                break;

            case AutomMode::SKILLS:
                if (skills_callback) { log.warn("Overwriting existing SKILLS callback"); }
                else { log.debug("Registering SKILLS callback"); }

                skills_callback = callback;
                break;

            case AutomMode::NONE:
                if (none_callback) { log.warn("Overwriting existing NONE callback"); }
                else { log.debug("Registering NONE callback"); }

                none_callback = callback;
                break;

            default:
                log.warn("Attempted registration with AutomMode::NONE");
            }
    }

    void select_autom(AutomMode mode, AutomSideColor side_color) {
        current_mode = mode;
        current_side = side_color;
        current_position = Convert::side_color_to_position(side_color);
        current_color = Convert::side_color_to_color(side_color);

        size_t index = static_cast<size_t>(side_color);
        switch (mode) {
            case AutomMode::QUALIFICATIONS: current_autom = qualification_methods[index]; break;
            case AutomMode::ELIMINATIONS: current_autom = elimination_methods[index]; break;
            case AutomMode::AWP: current_autom = awp_methods[index]; break;
            case AutomMode::SKILLS: current_autom = skills_callback; break;
            case AutomMode::NONE: current_autom = none_callback; break;
            default: current_autom = nullptr; break;

            if (!current_autom) {
                log.warn(std::format("No auton registered for {} {}",
                    Convert::to_string(mode),
                    Convert::to_string(side_color)
                ));
                return;
            }

            log.info(std::format("Selected {} {}", 
                Convert::to_string(mode), 
                Convert::to_string(side_color)
            ));
        }
    }

    void run_autom() {
        if (current_autom==nullptr) {
            log.warn(std::format("Automous mode {} has no callback", Convert::to_string(current_mode)));
            return;
        }
        log.info(std::format("Running {} {}", 
            Convert::to_string(current_mode),
            Convert::to_string(current_side)
        ));
        current_autom();
    }
}

namespace AutoManager::Convert {
    AutomSideColor to_side_color(AutomColor color, AutomPosition pos) {
        switch (color) {
            case AutomColor::RED: return (pos == AutomPosition::LEFT) ? AutomSideColor::RED_LEFT : AutomSideColor::RED_RIGHT;
            case AutomColor::BLUE: return (pos == AutomPosition::LEFT) ? AutomSideColor::BLUE_LEFT : AutomSideColor::BLUE_RIGHT;
            case AutomColor::COLOR_NONE: return AutomSideColor::NO_COLOR_AND_POSITION;
            default: return AutomSideColor::NO_COLOR_AND_POSITION;
        }
    }

    AutomColor side_color_to_color(AutomSideColor sc) {
        switch (sc) {
            case AutomSideColor::RED_LEFT:
            case AutomSideColor::RED_RIGHT: return AutomColor::RED;

            case AutomSideColor::BLUE_LEFT:
            case AutomSideColor::BLUE_RIGHT: return AutomColor::BLUE;

            case AutomSideColor::NO_COLOR_AND_POSITION: return AutomColor::COLOR_NONE;
            default: return AutomColor::COLOR_NONE;
        }
    }

    AutomPosition side_color_to_position(AutomSideColor sc) {
        switch (sc) {
            case AutomSideColor::RED_LEFT:
            case AutomSideColor::BLUE_LEFT: return AutomPosition::LEFT;

            case AutomSideColor::RED_RIGHT:
            case AutomSideColor::BLUE_RIGHT: return AutomPosition::RIGHT;

            case AutomSideColor::NO_COLOR_AND_POSITION: return AutomPosition::NO_POSITION;
            default: return AutomPosition::NO_POSITION;
        }
    }

    std::string_view to_string(AutomMode mode) {
        switch (mode) {
            case AutomMode::QUALIFICATIONS: return "Qualifications";
            case AutomMode::ELIMINATIONS: return "Eliminations";
            case AutomMode::AWP: return "AWP";
            case AutomMode::SKILLS: return "Skills";
            case AutomMode::NONE: return "None";
            default: return "Unknown";
        }
    }

    std::string_view to_string(AutomColor color) {
        switch (color) {
            case AutomColor::RED: return "Red";
            case AutomColor::BLUE: return "Blue";
            case AutomColor::COLOR_NONE: return "None";
            default: return "Unknown";
        }
    }

    std::string_view to_string(AutomPosition pos) {
        switch (pos) {
            case AutomPosition::LEFT: return "Left";
            case AutomPosition::RIGHT: return "Right";
            case AutomPosition::NO_POSITION: return "None";
            default: return "Unknown";
        }
    }

    std::string_view to_string(AutomSideColor sc) {
        switch (sc) {
            case AutomSideColor::RED_LEFT: return "Red Left";
            case AutomSideColor::RED_RIGHT: return "Red Right";
            case AutomSideColor::BLUE_LEFT: return "Blue Left";
            case AutomSideColor::BLUE_RIGHT: return "Blue Right";
            case AutomSideColor::NO_COLOR_AND_POSITION: return "None";
            default: return "Unknown";
        }
    }
}