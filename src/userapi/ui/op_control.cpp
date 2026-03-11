#include "userapi/ui/op_control.hpp"

#include "liblvgl/widgets/tabview/lv_tabview.h"
#include "liblvgl/core/lv_obj.h"

#include "userapi/ui/driver/sponsors.hpp"
#include "userapi/ui/driver/keybinds.hpp"
#include "userapi/ui/driver/debug.hpp"
#include "userapi/ui/driver/ports.hpp"
// #include "userapi/ui/driver/logs.hpp"

namespace ui::driver {
    lv_obj_t* driver_screen = lv_obj_create(NULL);
    
    void initialize() {
        lv_obj_t* tabview = lv_tabview_create(driver_screen);

        // Tabview style
        lv_obj_set_style_bg_color(tabview, lv_color_hex(0x191919), 0);
        lv_obj_set_style_text_color(tabview, lv_color_hex(0xffffff), 0);
        lv_tabview_set_tab_bar_size(tabview, 30);

        lv_obj_t* tab_btns = lv_tabview_get_tab_bar(tabview);
        lv_obj_set_style_bg_color(tab_btns, lv_color_hex(0x505050), 0);
        lv_obj_set_style_text_color(tab_btns, lv_color_hex(0xffffff), 0);

        // Create Tabs
        lv_obj_t* tabSponsors = lv_tabview_add_tab(tabview, "Sponsors");
        lv_obj_t* tabDebug = lv_tabview_add_tab(tabview, "Debug");
        // lv_obj_t* tabLogs = lv_tabview_add_tab(tabview, "Logs");
        lv_obj_t* tabKeybinds = lv_tabview_add_tab(tabview, "Keybinds");
        lv_obj_t* tabPorts = lv_tabview_add_tab(tabview, "Ports");

        // Initialization of all tabs
        ui::driver::sponsors::initialize(tabSponsors);
        // ui::driver::logs::initialize(tabLogs);
        ui::driver::debug::initialize(tabDebug);
        ui::driver::keybinds::initialize(tabKeybinds);
        ui::driver::ports::initialize(tabPorts);
    }
}