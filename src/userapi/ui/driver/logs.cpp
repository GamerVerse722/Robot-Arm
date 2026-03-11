#include "userapi/ui/driver/logs.hpp"

#include "liblvgl/widgets/label/lv_label.h"
#include "liblvgl/misc/lv_async.h"
#include "liblvgl/core/lv_obj.h"

#include <format>
#include <deque>

static std::deque<lv_obj_t*> logs_list;
static lv_obj_t* logsContainer;
static int maxLogs = 11;

namespace ui::driver::logs {
    void initialize(lv_obj_t* parent) {
        lv_obj_set_style_pad_all(parent, 0, LV_PART_MAIN);
        logsContainer = lv_obj_create(parent);
        lv_obj_set_size(logsContainer, lv_pct(100), lv_pct(100));
        lv_obj_set_flex_flow(logsContainer, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_scrollbar_mode(logsContainer, LV_SCROLLBAR_MODE_OFF);

        lv_obj_set_style_radius(logsContainer, 0, 0);
        lv_obj_set_style_pad_all(logsContainer, 2, 0);
        lv_obj_set_style_pad_row(logsContainer, 2, 0);
        lv_obj_set_style_border_width(logsContainer, 0, 0);
        lv_obj_set_style_bg_color(logsContainer, lv_color_hex(0x000000), 0);
        
        PROSLogger::Manager::subscribe(logs);
    }

    void logs(PROSLogger::LoggerEvent event) {
        // Capture data by value so it's safe
        std::string formatted = std::format("[{}] [{}] {}", event.time, levelToString(event.level), event.message);

        lv_async_call([](void* msg) {
            std::string* text = static_cast<std::string*>(msg);

            lv_obj_t* label = lv_label_create(logsContainer);
            lv_label_set_text(label, text->c_str());

            logs_list.push_back(label);

            if (logs_list.size() > maxLogs) {
                lv_obj_delete(logs_list.front());
                logs_list.pop_front();
            }

            lv_obj_scroll_to_y(logsContainer, lv_obj_get_scroll_bottom(logsContainer), LV_ANIM_OFF);

            delete text; // clean up
        }, new std::string(formatted)); // heap alloc because LVGL calls async later
    }
}