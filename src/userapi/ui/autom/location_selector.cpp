#include "userapi/ui/autom/location_selector.hpp"

#include "gamers-forge/proslogger.hpp"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/misc/lv_event.h"
#include "liblvgl/widgets/button/lv_button.h"
#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/display/lv_display.h"

#include <cstdint>
#include <format>

#include "userapi/ui/op_control.hpp"

namespace ui::autom::location_selector {
    PROSLogger::Logger log{"AutonomousLocationSelector"};

    void initialize() {
        log.info("Initializing Autonomous Location Selector Screen");
        lv_obj_set_size(location_screen, 480, 240);
        lv_obj_center(location_screen);

        lv_obj_set_style_pad_all(location_screen, 0, 0);
        lv_obj_set_style_pad_gap(location_screen, 2, 0);
        lv_obj_set_style_border_width(location_screen, 0, 0);

        static int32_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
        static int32_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
        lv_obj_set_grid_dsc_array(location_screen, col_dsc, row_dsc);
        lv_obj_set_layout(location_screen, LV_LAYOUT_GRID);

        register_button(AutomSideColor::RED_LEFT, 0, 0);
        register_button(AutomSideColor::RED_RIGHT, 0, 1);
        register_button(AutomSideColor::BLUE_RIGHT, 1, 0);
        register_button(AutomSideColor::BLUE_LEFT, 1, 1);

        log.info("Finished initialize of Autonomous Location Selector Screen");
    }

    void register_button(AutomSideColor color, int col, int row) {
        AutomColor autom_color = Convert::side_color_to_color(color);

        lv_obj_t* btn = lv_button_create(location_screen);
        lv_obj_set_grid_cell(
            btn,
            LV_GRID_ALIGN_STRETCH, col, 1,
            LV_GRID_ALIGN_STRETCH, row, 1
        );
        
        // Style Button
        lv_color_t bg_color;
        if (autom_color == AutomColor::RED) {bg_color = lv_palette_main(LV_PALETTE_RED);}
        else {bg_color = lv_palette_main(LV_PALETTE_BLUE);}

        lv_obj_set_style_bg_color(btn, bg_color, LV_PART_MAIN);
        lv_obj_set_style_radius(btn, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);

        // Buton Label
        lv_obj_t* label = lv_label_create(btn);
        lv_label_set_text(label, Convert::to_string(Convert::side_color_to_position(color)).data());
        lv_obj_set_style_text_color(label, lv_color_black(), LV_PART_MAIN);

        // Set Font Style
        static lv_style_t label_style;
        lv_style_init(&label_style);
        lv_style_set_text_font(&label_style, &lv_font_montserrat_30);
        lv_obj_add_style(label, &label_style, LV_PART_MAIN);

        lv_obj_center(label);

        // Add Callback
        lv_obj_add_event_cb(btn, button_event_handler, LV_EVENT_PRESSED, (void*)color);

        log.debug(std::format("Registered button for color: {}", Convert::to_string(autom_color)));
    }

    void button_event_handler(lv_event_t* e) {
        AutomSideColor side_color = (AutomSideColor)(intptr_t)lv_event_get_user_data(e);
        select_autom(active_mode, side_color);

        lv_screen_load_anim(ui::driver::driver_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, true);

        log.info(std::format("{} {} {} button was selected", Convert::to_string(active_mode), Convert::to_string(Convert::side_color_to_position(side_color)), Convert::to_string(Convert::side_color_to_color(side_color))));
    }
}