// #include "AP_Observer.h"

// // static変数の定義
// uint32_t AP_Observer::counter = 0;

// void AP_Observer::init() const {
//     // 初期化処理
// }

// void AP_Observer::update() const {
//     AP_Motors* motors = AP::motors();
//     if (!motors) {
//         gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: motors is nullptr!");
//         return;
//     }
    
//     // スロットル値取得と推力計算
//     float throttle_value = motors->get_throttle_out();
//     float thrust = -(THRUST_SCALE * throttle_value + THRUST_OFFSET) * g; // [N]
    
//     // 特異加速度取得（生データ）
//     Vector3f accel_body = AP::ins().get_accel();
    
//     // 外力計算 Fpb = m⋅fb - [0, 0, T]
//     Vector3f payload_force;
//     payload_force.x = UAV_mass * accel_body.x;                    // m⋅fb,x
//     payload_force.y = UAV_mass * accel_body.y;                    // m⋅fb,y  
//     payload_force.z = UAV_mass * accel_body.z - thrust;           // m⋅fb,z - T
    
//     // 送信頻度制御
//     counter++;
    
//     if (counter % 10 == 0) {
//         // 特異加速度と外力を送信
//         // gcs().send_text(MAV_SEVERITY_INFO, "ACCEL: X=%.3f Y=%.3f Z=%.3f", 
//         //                 accel_body.x, accel_body.y, accel_body.z);
//         gcs().send_text(MAV_SEVERITY_INFO, "FORCE: X=%.3f Y=%.3f Z=%.3f", 
//                         payload_force.x, payload_force.y, payload_force.z);
//     }
// }


#include "AP_Observer.h"

uint32_t AP_Observer::counter = 0;

void AP_Observer::init() const {
    force_filter.set_cutoff_frequency(SAMPLE_FREQ, CUTOFF_FREQ);
    force_filter.reset(Vector3f(0, 0, 0));
    filter_initialized = true;
    current_filtered_force.zero();
    last_update_ms = AP_HAL::millis();

    gcs().send_text(MAV_SEVERITY_INFO,
        "AP_Observer: Force filter initialized (%.1fHz cutoff)", CUTOFF_FREQ);
}

void AP_Observer::update() const {
    AP_Motors* motors = AP::motors();
    if (!motors) {
        gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: motors is nullptr!");
        return;
    }

    // 外力計算（既存コード）
    float throttle_value = motors->get_throttle_out();
    float thrust = -(THRUST_SCALE * throttle_value + THRUST_OFFSET) * g;
    Vector3f accel_body = AP::ins().get_accel();
    
    Vector3f payload_force;
    payload_force.x = UAV_mass * accel_body.x;
    payload_force.y = UAV_mass * accel_body.y;
    payload_force.z = UAV_mass * accel_body.z - thrust;

    // フィルタ適用と保存
    if (filter_initialized) {
        current_filtered_force = force_filter.apply(payload_force);
        last_update_ms = AP_HAL::millis();
    } else {
        current_filtered_force = payload_force;
    }

    // 送信頻度制御
    if ((++counter % 10) == 0) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "FILT_FORCE: X=%.3f Y=%.3f Z=%.3f",
            current_filtered_force.x, current_filtered_force.y, current_filtered_force.z);
    }
}
