// #include "AP_Observer.h"
// #include <AP_Motors/AP_Motors.h>

// void AP_Observer::update() {
//     uint32_t now = AP_HAL::millis();
//     if (now - _last_ms < PERIOD_MS) {
//         return;
//     }
//     _last_ms = now;

//     AP_Motors* motors = AP::motors();
//     if (!motors || !motors->armed()) {
//         hal.console->printf("AP_Observer: motors not armed\n");
//         return;
//     }

//     // 修正: motor_enabled 配列ではなく is_motor_enabled() を使う
//     for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
//         if (!motors->is_motor_enabled(i)) {
//             continue;
//         }
//         float thr_out = 0.0f;
//         motors->get_thrust(i, thr_out);
//         hal.console->printf("Motor[%u] Thrust: %.3f  ", i, thr_out);
//     }
//     hal.console->printf("\n");
// }

#include "AP_Observer.h"

void AP_Observer::init() {
    // 初期化処理（空でも可）
}

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

    // 送信頻度制限用の静的カウンター
    static uint8_t counter = 0;
    counter++;

    // 推力データを文字列として構築
    char thrust_msg[200];
    int pos = 0;

    // 修正: motor_enabled 配列ではなく is_motor_enabled() を使う
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (!motors->is_motor_enabled(i)) {
            continue;
        }
        float thr_out = 0.0f;
        motors->get_thrust(i, thr_out);
        
        // コンソール出力（従来通り）
        hal.console->printf("Motor[%u] Thrust: %.3f  ", i, thr_out);
        
        // 文字列に追加（MAVLink送信用）
        pos += snprintf(thrust_msg + pos, sizeof(thrust_msg) - pos, 
                       "M%u:%.3f ", i, thr_out);
    }
    hal.console->printf("\n");

    // 50回に1回MAVLink経由でデバッグメッセージを送信
    if (counter >= 50) {
        counter = 0;  // カウンターリセット
        gcs().send_text(MAV_SEVERITY_INFO, "Thrust: %s", thrust_msg);
    }
}
