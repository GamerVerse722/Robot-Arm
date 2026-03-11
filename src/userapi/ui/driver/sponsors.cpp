#include "userapi/ui/driver/sponsors.hpp"

#include "liblvgl/widgets/image/lv_image.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/core/lv_obj.h"

#include <vector>

// LV_IMAGE_DECLARE(EBA_Automotive_Auto_Repair);
// LV_IMAGE_DECLARE(PioneerContainerService);

static std::vector<lv_image_dsc_t> sponsors_list = {
    // EBA_Automotive_Auto_Repair, 
    // PioneerContainerService
};
static int currentSponsorIndex = 0;
static lv_obj_t* sponsorImage;

namespace ui::driver::sponsors {
    void initialize(lv_obj_t* parent) {
        lv_obj_set_style_pad_all(parent, 0, LV_PART_MAIN);

        sponsorImage = lv_image_create(parent);
        lv_obj_align(sponsorImage, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_image_set_src(sponsorImage, &sponsors_list[0]);

        lv_timer_create(image_render_timer, 5000, nullptr);
    }
    
    void image_render_timer(lv_timer_t *timer) {
        lv_image_set_src(sponsorImage, &sponsors_list[currentSponsorIndex]);
        currentSponsorIndex = (currentSponsorIndex + 1) % sponsors_list.size();
    }
}