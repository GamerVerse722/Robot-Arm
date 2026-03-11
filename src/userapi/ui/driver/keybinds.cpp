#include "userapi/ui/driver/keybinds.hpp"

#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/core/lv_obj.h"

#include <format>

static lv_obj_t* keybindsList;

namespace ui::driver::keybinds {
    void initialize(lv_obj_t* parent) {
        lv_obj_set_style_pad_all(parent, 0, LV_PART_MAIN);
        keybindsList = lv_obj_create(parent);
        lv_obj_set_size(keybindsList, lv_pct(100), lv_pct(100));
        lv_obj_set_flex_flow(keybindsList, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_scroll_dir(keybindsList, LV_DIR_VER);

        lv_obj_set_style_radius(keybindsList, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(keybindsList, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_row(keybindsList, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(keybindsList, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_color(keybindsList, lv_color_hex(0x000000), LV_PART_MAIN);

        create_category("Drive");
        create_row("Y-A", "Changes the Driving Modes.");

        create_category("Scrapper");
        create_row("B", "Toggle the scrapper.");

        create_category("Splitter");
        create_row("X", "Toggle Splitter up and down.");

        create_category("Descore");
        create_row("L2", "Toggle Descore up and down.");

        create_category("Load");
        create_row("L1", "Loads balls without scoring.");
        create_row("Y-L1", "Toggle loads balls without scoring.");

        create_category("Score");
        create_row("R1", "Moves balls into low score.");
        create_row("Y-R1", "Toggle move balls into low score.");
        create_row("R2", "Moves balls into high/middle score.");
        create_row("Y-R2", "Toggle moves balls into high/middle score.");
    }

    void create_category(std::string category) {
        lv_obj_t* category_container = lv_obj_create(keybindsList);
        lv_obj_set_size(category_container, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_radius(category_container, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(category_container, 4, LV_PART_MAIN);
        lv_obj_set_style_border_width(category_container, 1, LV_PART_MAIN);
        lv_obj_set_style_border_side(category_container, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
        lv_obj_set_style_border_color(category_container, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_border_post(category_container, true, LV_PART_MAIN);
        lv_obj_set_style_bg_color(category_container, lv_color_hex(0x3f444c), LV_PART_MAIN);

        lv_obj_t* categoryLabel = lv_label_create(category_container);
        lv_label_set_text(categoryLabel, category.c_str());
    }

    void create_row(std::string key, std::string description) {
        lv_obj_t* row = lv_obj_create(keybindsList);
        lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_radius(row, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(row, 4, LV_PART_MAIN);
        lv_obj_set_style_border_width(row, 1, LV_PART_MAIN);
        lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
        lv_obj_set_style_border_color(row, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_border_post(row, true, LV_PART_MAIN);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); 

        lv_obj_t *id = lv_label_create(row);
        lv_label_set_text(id, std::format("Id: {}", key).c_str());

        lv_obj_t *desc = lv_label_create(row);
        lv_label_set_text(desc, std::format("\"{}\"", description).c_str());
    }
}