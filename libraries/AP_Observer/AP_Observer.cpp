#include "AP_Observer.h"

void AP_Observer::init() const {
    hal.console->printf("AP_Observer initialized\n");
}

void AP_Observer::update() const {
    AP_Motors* motors = AP::motors();
    if (!motors) {
        gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: motors is nullptr!");
        return;
    }
    
    // モーターマスクを取得してモーター数をカウント
    uint32_t motor_mask = motors->get_motor_mask();
    uint8_t motor_count = __builtin_popcount(motor_mask);
    
    // 直接フォーマット文字列で送信
    gcs().send_text(MAV_SEVERITY_INFO, "Motor count: %d", motor_count);
}
