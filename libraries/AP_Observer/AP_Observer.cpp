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
    
    // モーターマスクを取得
    uint32_t motor_mask = motors->get_motor_mask();
    
    // モーター値を格納する配列
    float motor_values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t motor_count = 0;
    
    // 各モーターのスラスト値を取得
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS && motor_count < 4; i++) {
        if (motor_mask & (1U << i)) {
            motors->get_thrust(i, motor_values[motor_count]);
            motor_count++;
        }
    }
    
    // 一行でモーター値を表示
    gcs().send_text(MAV_SEVERITY_INFO, "Motors: M0:%d%% M1:%d%% M2:%d%% M3:%d%%", 
                  (int)(motor_values[0] * 100),
                  (int)(motor_values[1] * 100),
                  (int)(motor_values[2] * 100),
                  (int)(motor_values[3] * 100));
}
