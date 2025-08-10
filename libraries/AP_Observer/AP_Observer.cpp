// AP_Observer.cpp
#include "AP_Observer.h"

AP_Observer ap_observer;
uint32_t AP_Observer::counter = 0;

// パラメータテーブル定義
const AP_Param::GroupInfo AP_Observer::var_info[] = {
    // @Param: CORR_GAIN
    // @DisplayName: Observer Correction Gain
    // @Description: Gain for attitude correction based on external force estimation
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("CORR_GAIN", 0, AP_Observer, _correction_gain, 0.3f),

    AP_GROUPEND
};

void AP_Observer::init() const {
    current_filtered_force.zero();
    current_correction_quat = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    last_update_ms = AP_HAL::millis();
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: initialized");
}

void AP_Observer::update() const {
    AP_Motors* motors = AP::motors();
    if (!motors) {
        gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: motors nullptr");
        return;
    }

    float throttle = motors->get_throttle_out();
    float thrust   = -(THRUST_SCALE * throttle + THRUST_OFFSET) * g;
    Vector3f accel = AP::ins().get_accel();
    Vector3f payload;
    payload.x = UAV_mass * accel.x;
    payload.y = UAV_mass * accel.y;
    payload.z = UAV_mass * accel.z - thrust;

    current_filtered_force  = payload;
    current_correction_quat = calculate_correction_from_force(payload);
    last_update_ms          = AP_HAL::millis();

    if ((++counter % 10) == 0) {
        // EF出力
        gcs().send_text(MAV_SEVERITY_INFO,
            "EF=%.3f,%.3f,%.3f",
            current_filtered_force.x,
            current_filtered_force.y,
            current_filtered_force.z
        );
        // Q出力
        gcs().send_text(MAV_SEVERITY_INFO,
            "Q=%.6f,%.6f,%.6f,%.6f",
            current_correction_quat.q1,
            current_correction_quat.q2,
            current_correction_quat.q3,
            current_correction_quat.q4
        );
        // 補正ゲイン出力
        gcs().send_text(MAV_SEVERITY_INFO,
            "Gain=%.2f",
            _correction_gain.get()
        );
    }
}

Quaternion AP_Observer::calculate_correction_from_force(const Vector3f& force) const {
    float mag = force.length();
    if (mag < FORCE_THRESHOLD) {
        return Quaternion(1, 0, 0, 0);
    }

    // パラメータ化したゲインを取得
    float correction_gain = _correction_gain.get();
    float roll  =  force.y * correction_gain / UAV_mass;
    float pitch =  force.x * correction_gain / UAV_mass;

    roll  = constrain_float(roll,  -MAX_CORRECTION_ANGLE, MAX_CORRECTION_ANGLE);
    pitch = constrain_float(pitch, -MAX_CORRECTION_ANGLE, MAX_CORRECTION_ANGLE);

    Quaternion q;
    q.from_euler(roll, pitch, 0.0f);
    q.normalize();
    return q;
}
