#include "AP_Observer.h"
#include <AP_Motors/AP_Motors.h>

void AP_Observer::update() {
    uint32_t now = AP_HAL::millis();
    if (now - _last_ms < PERIOD_MS) {
        return;
    }
    _last_ms = now;

    AP_Motors* motors = AP::motors();
    if (!motors || !motors->armed()) {
        hal.console->printf("AP_Observer: motors not armed\n");
        return;
    }

    // 修正: motor_enabled 配列ではなく is_motor_enabled() を使う
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (!motors->is_motor_enabled(i)) {
            continue;
        }
        float thr_out = 0.0f;
        motors->get_thrust(i, thr_out);
        hal.console->printf("Motor[%u] Thrust: %.3f  ", i, thr_out);
    }
    hal.console->printf("\n");
}
