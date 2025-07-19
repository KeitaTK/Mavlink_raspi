#include "AP_Observer.h"
#include <AP_HAL/AP_HAL.h>

void AP_Observer::init() const {
    hal.console->printf("AP_Observer initialized\n");
}

void AP_Observer::update() const {
    AP_Motors* motors = AP::motors();
    if (!motors) {
        gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: motors is nullptr!");
        return;
    }
    char msg[128];
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        float thr_out = 0.0f;
        motors->get_thrust(i, thr_out);
        // メッセージを組み立てて送信
        snprintf(msg, sizeof(msg), "Motor%d thrust: %f", i, thr_out);
        gcs().send_text(MAV_SEVERITY_INFO, msg);
    }
}