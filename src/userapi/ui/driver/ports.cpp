#include "userapi/ui/driver/ports.hpp"

#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/core/lv_obj.h"

#include <format>

static lv_obj_t* portsList;

namespace ui::driver::ports {
    void initialize(lv_obj_t* parent) {
        lv_obj_set_style_pad_all(parent, 0, LV_PART_MAIN);
        portsList = lv_obj_create(parent);
        lv_obj_set_size(portsList, lv_pct(100), lv_pct(100));
        lv_obj_set_flex_flow(portsList, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_scroll_dir(portsList, LV_DIR_VER);

        lv_obj_set_style_radius(portsList, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(portsList, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_row(portsList, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(portsList, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_color(portsList, lv_color_hex(0x000000), LV_PART_MAIN);

        create_category("Drivetrain");
        create_row(5, "Motor", "Left Motor Group");
        create_row(-6, "Motor", "Left Motor Group");
        create_row(-8, "Motor", "Left Motor Group");
        create_row(-2, "Motor", "Right Motor Group");
        create_row(3, "Motor", "Right Motor Group");
        create_row(4, "Motor", "Right Motor Group");

        create_category("Intake");
        create_row(-18, "Motor", "Intake");
        create_row(19, "Motor", "Intake");
        create_row(15, "Motor", "Top Loader");

        create_category("Sensors");
        create_row(11, "Optical", "Color Sort");

        create_category("Pneumatics");
        create_row(5, "Pneumatics", "Splitter");
        create_row(6, "Pneumatics", "Scrapper");
        create_row(8, "Pneumatics", "Descore");

        create_category("Odometry");
        create_row(9, "Imu", "Internal Sensor");

    }

    void create_category(std::string category) {
        lv_obj_t* category_container = lv_obj_create(portsList);
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

    void create_row(int port_number, std::string port_type, std::string description) {
        lv_obj_t* row = lv_obj_create(portsList);
        lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_radius(row, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(row, 4, LV_PART_MAIN);
        lv_obj_set_style_border_width(row, 1, LV_PART_MAIN);
        lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
        lv_obj_set_style_border_color(row, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_border_post(row, true, LV_PART_MAIN);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); 

        lv_obj_t *port = lv_label_create(row);
        lv_label_set_text(port, std::format("Port: {}", port_number).c_str());

        lv_obj_t *type = lv_label_create(row);
        lv_label_set_text(type, std::format("\"{}\"", port_type).c_str());

        lv_obj_t *desc = lv_label_create(row);
        lv_label_set_text(desc, std::format("\"{}\"", description).c_str());
    }
}