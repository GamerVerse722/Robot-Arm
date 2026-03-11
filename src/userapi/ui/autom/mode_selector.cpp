#include "userapi/ui/autom/mode_selector.hpp"

#include "gamers-forge/proslogger.hpp"
#include "liblvgl/misc/lv_event.h"
#include "liblvgl/misc/lv_palette.h"
#include "liblvgl/widgets/button/lv_button.h"
#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/display/lv_display.h"

#include "userapi/ui/autom/location_selector.hpp"
#include "userapi/ui/op_control.hpp"

#include <format>


namespace ui::autom::mode_selector {
    PROSLogger::Logger log{"AutonomousModeSelector"};

    void initialize() {
        log.info("Initializing Autonomous Mode Selector Screen");
        lv_obj_set_size(mode_screen, 480, 240);
        lv_obj_center(mode_screen);

        lv_obj_set_style_pad_all(mode_screen, 0, 0);
        lv_obj_set_style_pad_gap(mode_screen, 2, 0);
        lv_obj_set_style_border_width(mode_screen, 0, 0);

        static int32_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
        static int32_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
        lv_obj_set_grid_dsc_array(mode_screen, col_dsc, row_dsc);
        lv_obj_set_layout(mode_screen, LV_LAYOUT_GRID);

        register_button(AutomMode::QUALIFICATIONS, 0);
        register_button(AutomMode::ELIMINATIONS, 1);
        register_button(AutomMode::SKILLS, 2);
        register_button(AutomMode::NONE, 3);

        log.info("Finished initialize of Autonomous Mode Selector Screen");
    }
    
    void register_button(AutomMode mode, int row) {
        lv_obj_t* btn = lv_button_create(mode_screen);
        lv_obj_set_grid_cell(
            btn,
            LV_GRID_ALIGN_STRETCH, 0, 1,
            LV_GRID_ALIGN_STRETCH, row, 1
        );
        
        // Style Button
        lv_color_t bg_color;
        switch (mode % 2) {
            case 0: {bg_color = lv_palette_main(LV_PALETTE_RED); break;}
            case 1: {bg_color = lv_palette_main(LV_PALETTE_BLUE); break;}
        }

        lv_obj_set_style_bg_color(btn, bg_color, LV_PART_MAIN);
        lv_obj_set_style_radius(btn, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);

        // Buton Label
        lv_obj_t* label = lv_label_create(btn);
        lv_label_set_text(label, Convert::to_string(mode).data());
        lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);

        // Set Font Style
        static lv_style_t label_style;
        lv_style_init(&label_style);
        lv_style_set_text_font(&label_style, &lv_font_montserrat_30);
        lv_obj_add_style(label, &label_style, LV_PART_MAIN);

        // Add Callback
        lv_obj_add_event_cb(btn, button_event_handler, LV_EVENT_PRESSED, (void*)mode);

        log.debug(std::format("Registered {0} Button", Convert::to_string(mode)));
    }

    void button_event_handler(lv_event_t* e) {
        AutomMode mode = (AutomMode)(intptr_t)lv_event_get_user_data(e);
        switch (mode) {
            case AutomMode::QUALIFICATIONS: {
                location_selector::active_mode = AutomMode::QUALIFICATIONS;
                lv_screen_load(location_selector::location_screen);
                break;
            }
            case AutomMode::ELIMINATIONS: {
                location_selector::active_mode = AutomMode::ELIMINATIONS;
                lv_screen_load(location_selector::location_screen);
                break;
            }
            case AutomMode::AWP: {
                location_selector::active_mode = AutomMode::AWP;
                lv_screen_load(location_selector::location_screen);
                break;
            }
            case AutomMode::SKILLS: {
                AutoManager::select_autom(AutomMode::SKILLS, AutomSideColor::NO_COLOR_AND_POSITION);
                lv_screen_load_anim(ui::driver::driver_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, true);
                break;
            }
            case AutomMode::NONE: {
                AutoManager::select_autom(AutomMode::NONE, AutomSideColor::NO_COLOR_AND_POSITION);
                lv_screen_load_anim(ui::driver::driver_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, true);
                break;
            }
            break;
        }

        log.info(std::format("{0} button was selected", Convert::to_string(mode)));
    }
}