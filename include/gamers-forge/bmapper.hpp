#pragma once

#include "gamers-forge/proslogger.hpp"

#include "pros/misc.hpp"

#include <functional>
#include <optional>
#include <string>
#include <map>
#include <set>

namespace BMapper {
    class ButtonHandler;

    typedef std::function<void()> keybind_method_t;

    typedef struct keybind_actions_s {
        keybind_method_t onPress = nullptr;
        keybind_method_t onHold = nullptr;
        keybind_method_t onRelease = nullptr;

        bool onPressTask = false;
        bool onHoldTask = false;
        bool onReleaseTask = false;
    } keybind_actions_s_t;

    typedef struct keybind_state {
        bool isPressed = false;
        bool wasPressed = false;
        bool isHeld = false;
    } keybind_state_s_t;

    typedef struct keybind_s {
        std::optional<pros::controller_digital_e_t> action_key = std::nullopt;
        keybind_actions_s_t actions;
        keybind_state_s_t state;
        std::string id = "";
    } keybind_s_t;
    
    inline std::string keyToShort(pros::controller_digital_e_t key) {
        switch (key) {
            case pros::E_CONTROLLER_DIGITAL_L1: return "L1";
            case pros::E_CONTROLLER_DIGITAL_L2: return "L2";
            case pros::E_CONTROLLER_DIGITAL_R1: return "R1";
            case pros::E_CONTROLLER_DIGITAL_R2: return "R2";
            case pros::E_CONTROLLER_DIGITAL_UP: return "U";
            case pros::E_CONTROLLER_DIGITAL_DOWN: return "D";
            case pros::E_CONTROLLER_DIGITAL_LEFT: return "L";
            case pros::E_CONTROLLER_DIGITAL_RIGHT: return "R";
            case pros::E_CONTROLLER_DIGITAL_A: return "A";
            case pros::E_CONTROLLER_DIGITAL_B: return "B";
            case pros::E_CONTROLLER_DIGITAL_X: return "X";
            case pros::E_CONTROLLER_DIGITAL_Y: return "Y";
            default: return "??";
        }
    }
    
    class KeybindBuilder {
        private:
            std::optional<pros::controller_digital_e_t> actionKey;
            BMapper::keybind_actions_s actions;
            pros::controller_digital_e_t key;
            BMapper::ButtonHandler& handler;
            bool applied = false;
            
            PROSLogger::Logger log{"KeybindBuilder"};
            std::string category = "Uncategorized";

        public:
            KeybindBuilder(
                pros::controller_digital_e_t key,
                BMapper::ButtonHandler& handler,
                std::optional<pros::controller_digital_e_t> modifier = std::nullopt);

            ~KeybindBuilder();

            KeybindBuilder& onPress(keybind_method_t callback, bool tasked = false);

            KeybindBuilder& onHold(keybind_method_t callback, bool tasked = false);

            KeybindBuilder& onRelease(keybind_method_t callback, bool tasked = false);

            KeybindBuilder& setCategory(std::string category);

            void apply();
    };

    class ButtonHandler {
        private:
            template <typename... Buttons>bool areKeysPressed(Buttons... buttons);
            std::map<pros::controller_digital_e_t, keybind_s_t> action_keybinds;
            std::map<pros::controller_digital_e_t, keybind_s_t> keybinds;
            std::set<pros::controller_digital_e_t> register_key_set;
            std::set<std::string> register_id_set;
            pros::Controller& controller;
            bool activated = false;
            int delay = 10;

            PROSLogger::Logger log{"ButtonHandler"};

            void registerKeybind(
                pros::controller_digital_e_t key,
                keybind_actions_s_t keybind_actions,
                std::optional<pros::controller_digital_e_t> action_key = std::nullopt,
                std::string category = "Uncategorized");

            void executeAction(const std::string& name, const std::string& id, keybind_method_t action, bool runAsTask);
            void handleKeybind(pros::controller_digital_e_t key);
            void handleActionKeybind(pros::controller_digital_e_t key);

            void update(pros::controller_digital_e_t key);
            void run(pros::controller_digital_e_t key);

            friend class KeybindBuilder;

        public:
            /**
             * @brief Construct a new Button Handler object
             * 
             * @param controller The controller to listen to
             */
            ButtonHandler(pros::Controller& controller) : controller(controller) {};

            /**
             * @brief KeybindBuilder method lets user create keybinds easily
             * 
             * @param key 
             * @param modifier 
             * @return KeybindBuilder 
             */
            KeybindBuilder bind(
                pros::controller_digital_e_t key, 
                std::optional<pros::controller_digital_e_t> modifier = std::nullopt) {
                return KeybindBuilder(key, *this, modifier);
            }

            /** Start the button handler */
            void start();

            /** Stop the button handler */
            void stop();

            /** @param interval in milliseconds */
            void setDelay(int interval);

            /** @return interval in milliseconds */
            int getDelay() const;
            
            /** @return registered keybinds ids */
            std::set<std::string> getKeybindIds();

            /** Reset all keybinds */
            void reset();
        };
}
