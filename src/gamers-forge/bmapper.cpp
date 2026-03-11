#include "gamers-forge/bmapper.hpp"

#include "pros/rtos.hpp"

#include <format>

namespace BMapper {
    KeybindBuilder::KeybindBuilder(pros::controller_digital_e_t key, BMapper::ButtonHandler& handler, std::optional<pros::controller_digital_e_t> modifier): key(key), handler(handler), actionKey(modifier) {
        if (modifier.has_value()) {
            log.info(std::format("Created Keybind Builder: {}-{}", keyToShort(key), keyToShort(modifier.value())));
        } else {
            log.info("Created Keybind Builder: " + keyToShort(key));
        }
    }

    KeybindBuilder::~KeybindBuilder() {
        if (!applied) {
            handler.registerKeybind(key, actions, actionKey, this->category);
            log.info("Keybind applied in destructor");
        }
    }

    KeybindBuilder& KeybindBuilder::onPress(keybind_method_t callback, bool tasked) {
        log.debug("onPress callback set.");
        actions.onPress = callback;
        return *this;
    }

    KeybindBuilder& KeybindBuilder::onHold(keybind_method_t callback, bool tasked) {
        log.debug("onHold callback set.");
        actions.onHold = callback;
        return *this;
    }

    KeybindBuilder& KeybindBuilder::onRelease(keybind_method_t callback, bool tasked) {
        log.debug("onRelease callback set.");
        actions.onRelease = callback;
        return *this;
    }

    KeybindBuilder& KeybindBuilder::setCategory(std::string category) {
        log.info("Category set to: " + category);
        this->category = category;
        return *this;
    }

    void KeybindBuilder::apply() {
        applied = true;
        handler.registerKeybind(key, actions, actionKey, this->category);
        log.info("Keybind applied explicitly");
    }

    void ButtonHandler::registerKeybind(pros::controller_digital_e_t key, keybind_actions_s_t keybind_actions, std::optional<pros::controller_digital_e_t> action_key, std::string category) {
        this->register_key_set.insert(key);

        keybind_s_t create_keybind;
        create_keybind.action_key = action_key;
        create_keybind.actions = keybind_actions;
        create_keybind.state = keybind_state_s_t{false, false, false};
        
        if (action_key.has_value()) {
            create_keybind.id = std::format("{}/{}-{}", category, keyToShort(key), keyToShort(action_key.value()));
            this->action_keybinds[key] = create_keybind;

        } else {
            create_keybind.id = std::format("{}/{}", category, keyToShort(key));
            this->keybinds[key] = create_keybind;
        }

        this->register_id_set.insert(create_keybind.id);
        log.info("Registered Keybind: " + create_keybind.id);
    }

    template <typename... Buttons> bool ButtonHandler::areKeysPressed(Buttons... buttons) {
        return (... && this->controller.get_digital(buttons));
    }

    void ButtonHandler::update(pros::controller_digital_e_t key) {
        if (this->action_keybinds.contains(key)) {
            keybind_s_t& action_keybind = this->action_keybinds[key];

            action_keybind.state.wasPressed = action_keybind.state.isPressed;
            action_keybind.state.isPressed = this->areKeysPressed(key, action_keybind.action_key.value());
            action_keybind.state.isHeld = action_keybind.state.isPressed && action_keybind.state.wasPressed;
        }

        if (this->keybinds.contains(key)) {
            keybind_s_t& keybind = this->keybinds[key];

            keybind.state.wasPressed = keybind.state.isPressed;
            keybind.state.isPressed = this->controller.get_digital(key);
            if (this->action_keybinds.contains(key)) {
                keybind.state.isPressed = keybind.state.isPressed && !this->controller.get_digital(this->action_keybinds[key].action_key.value());
            }
            keybind.state.isHeld = keybind.state.isPressed && keybind.state.wasPressed;
        }
    }

    void ButtonHandler::executeAction(const std::string& name, const std::string& id, keybind_method_t action, bool runAsTask) {
        if (!action) return;

        log.debug(std::format("Keybind {} {}", id, name));

        if (runAsTask) {
            pros::Task([=]{action();});
        } else {
            action();
        }
    }

    void ButtonHandler::handleKeybind(pros::controller_digital_e_t key) {
        keybind_s_t keybind = this->keybinds[key];
        keybind_state_s_t state = keybind.state;
        keybind_actions_s_t action = keybind.actions;

        if (state.isPressed && !state.wasPressed) {
            if (this->action_keybinds.contains(key) && !this->action_keybinds[key].state.isPressed && this->action_keybinds[key].state.wasPressed) {
            } else {
                executeAction("onPress", keybind.id, action.onPress, action.onPressTask);
            }
        }
        if (state.isHeld) {
            executeAction("onHold", keybind.id, action.onHold, action.onHoldTask);
        }
        if (!state.isPressed && state.wasPressed) {
            executeAction("onRelease", keybind.id, action.onRelease, action.onReleaseTask);
        }
    }

    void ButtonHandler::handleActionKeybind(pros::controller_digital_e_t key) {
        keybind_s_t keybind_action = this->action_keybinds[key];
        keybind_state_s_t state = keybind_action.state;
        keybind_actions_s_t action = keybind_action.actions;

        if (state.isPressed && !state.wasPressed) {
            executeAction("onPress", keybind_action.id, action.onPress, action.onPressTask);
        }
        if (state.isHeld) {
            executeAction("onHold", keybind_action.id, action.onHold, action.onHoldTask);
        }
        if (!state.isPressed && state.wasPressed) {
            executeAction("onRelease", keybind_action.id, action.onRelease, action.onReleaseTask);

            if (this->keybinds.contains(key) && this->keybinds[key].state.isPressed && !this->keybinds[key].state.wasPressed) {
                keybind_s_t& keybind = this->keybinds[key];
                executeAction("onPress", keybind.id, keybind.actions.onPress, keybind.actions.onPressTask);
            }
        }
    }

    void ButtonHandler::run(pros::controller_digital_e_t key) {
        if (this->keybinds.contains(key)) {
            handleKeybind(key);
        }

        if (this->action_keybinds.contains(key)) {
            handleActionKeybind(key);
        }
    }
    
    void ButtonHandler::start() {
        this->activated = true;
        log.info("ButtonHandler started");
        pros::Task button_task([&] {
            while (this->activated) {
                for (auto key : this->register_key_set) {
                    this->update(key);
                    this->run(key);
                }
                pros::delay(this->delay);
            }
        });
    }

    void ButtonHandler::stop() {
        this->activated = false;
        log.info("ButtonHandler stopped");
    }

    void ButtonHandler::setDelay(int interval) {
        log.debug("ButtonHandler delay set to " + std::to_string(interval) + "ms");
        this->delay = interval;
    }

    int ButtonHandler::getDelay() const {
        return this->delay;
    }

    void ButtonHandler::reset() {
        this->keybinds.clear();
        this->action_keybinds.clear();
        this->register_key_set.clear();
        log.info("All keybinds cleared");
    }
}