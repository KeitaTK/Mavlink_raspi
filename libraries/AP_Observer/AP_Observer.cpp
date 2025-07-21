#include "AP_Observer.h"

void AP_Observer::init() const {
    // gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer initialized");
}

void AP_Observer::update() const {
    AP_Motors* motors = AP::motors();
    if (!motors) {
        gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: motors is nullptr!");
        return;
    }
    
    // スロットル値取得
    float throttle_value = motors->get_throttle_out();
    float paylode = THRUST_SCALE*throttle_value + THRUST_OFFSET + UAV_mass; //kg
    
    // gcs().send_text(MAV_SEVERITY_INFO, "THROTTLE: %.4f", paylode);
    (void)paylode; // 未使用変数の警告を回避
}
