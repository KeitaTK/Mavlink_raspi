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
    
    // モーターマスクを取得
    uint32_t motor_mask = motors->get_motor_mask();
    
    // モーター値を格納する配列
    float motor_values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t motor_count = 0;
    
    // 各モーターのスラスト値を取得.推力に線形変換
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS && motor_count < 4; i++) {
        if (motor_mask & (1U << i)) {
            motors->get_thrust(i, motor_values[motor_count]);
            motor_values[motor_count] = THRUST_SCALE * motor_values[motor_count] + THRUST_OFFSET;
            motor_count++;
        }
    }

    // 推力値
    gcs().send_text(MAV_SEVERITY_INFO, "Thrust: M0:%.3f M1:%.3f M2:%.3f M3:%.3f", 
                motor_values[0],
                motor_values[1], 
                motor_values[2],
                motor_values[3]);
}