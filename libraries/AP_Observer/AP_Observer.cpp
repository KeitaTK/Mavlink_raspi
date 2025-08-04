#include "AP_Observer.h"

AP_Observer ap_observer;

uint32_t AP_Observer::counter = 0;

void AP_Observer::init() const {
    force_filter.set_cutoff_frequency(SAMPLE_FREQ, CUTOFF_FREQ);
    force_filter.reset(Vector3f(0, 0, 0));
    filter_initialized = true;
    current_filtered_force.zero();
    current_correction_quat = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // 単位クオータニオンで初期化
    last_update_ms = AP_HAL::millis();

    gcs().send_text(MAV_SEVERITY_INFO,
        "AP_Observer: Force filter and correction quaternion initialized (%.1fHz cutoff)", CUTOFF_FREQ);
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
        
        // 補正クオータニオンを計算
        current_correction_quat = calculate_correction_from_force(current_filtered_force);
        
        last_update_ms = AP_HAL::millis();
    } else {
        current_filtered_force = payload_force;
        current_correction_quat = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // 単位クオータニオン
    }

    // 送信頻度制御（補正クオータニオン情報も追加）
    if ((++counter % 10) == 0) {
        // オイラー角に変換してログ出力
        float roll, pitch, yaw;
        current_correction_quat.to_euler(roll, pitch, yaw);
        
        gcs().send_text(MAV_SEVERITY_INFO,
            "CORRECTION: Force=[%.3f,%.3f,%.3f] Quat_RPY=[%.2f,%.2f,%.2f]deg",
            current_filtered_force.x, current_filtered_force.y, current_filtered_force.z,
            degrees(roll), degrees(pitch), degrees(yaw));
    }
}

Quaternion AP_Observer::calculate_correction_from_force(const Vector3f& force) const {
    // 外力の大きさをチェック
    float force_magnitude = force.length();
    if (force_magnitude < FORCE_THRESHOLD) {
        // 外力が小さい場合は補正なし（単位クオータニオン）
        return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }

    // 外力に基づいた補正角度を計算
    // 例：水平面の外力に対してロール・ピッチ補正を生成
    float correction_roll = -force.y * CORRECTION_GAIN / UAV_mass;   // Y軸外力 → ロール補正
    float correction_pitch = force.x * CORRECTION_GAIN / UAV_mass;   // X軸外力 → ピッチ補正
    float correction_yaw = 0.0f;  // ヨー補正は今回は実装しない

    // 補正角度を制限
    correction_roll = constrain_float(correction_roll, -MAX_CORRECTION_ANGLE, MAX_CORRECTION_ANGLE);
    correction_pitch = constrain_float(correction_pitch, -MAX_CORRECTION_ANGLE, MAX_CORRECTION_ANGLE);

    // オイラー角からクオータニオンを生成（正しい方法）
    Quaternion correction_quat;
    correction_quat.from_euler(correction_roll, correction_pitch, correction_yaw);
    return correction_quat;
}
