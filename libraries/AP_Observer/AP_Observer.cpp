// AP_Observer.cpp - 完全最適化版
#include "AP_Observer.h"

AP_Observer ap_observer;
uint32_t AP_Observer::counter = 0;

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
    float thrust = -(THRUST_SCALE * throttle + THRUST_OFFSET) * g;
    Vector3f accel = AP::ins().get_accel();
    Vector3f payload;
    payload.x = UAV_mass * accel.x;
    payload.y = UAV_mass * accel.y;
    payload.z = UAV_mass * accel.z - thrust;

    current_filtered_force = payload;
    current_correction_quat = calculate_correction_from_force(current_filtered_force);
    last_update_ms = AP_HAL::millis();

    if ((++counter % 10) == 0) {
        // 推定外力（EF）を1行目で送信
        gcs().send_text(MAV_SEVERITY_INFO,
            "EF=%.3f,%.3f,%.3f",
            current_filtered_force.x,
            current_filtered_force.y,
            current_filtered_force.z
        );
        
        // 補正クオータニオン（Q）を2行目で送信
        gcs().send_text(MAV_SEVERITY_INFO,
            "Q=%.6f,%.6f,%.6f,%.6f",
            current_correction_quat.q1,
            current_correction_quat.q2,
            current_correction_quat.q3,
            current_correction_quat.q4
        );
    }
}

Quaternion AP_Observer::calculate_correction_from_force(const Vector3f& force) const {
    float mag = force.length();
    
    if (mag < FORCE_THRESHOLD) {
        return Quaternion(1, 0, 0, 0);
    }

    float roll  = -force.y * CORRECTION_GAIN / UAV_mass;
    float pitch =  force.x * CORRECTION_GAIN / UAV_mass;

    roll  = constrain_float(roll,  -MAX_CORRECTION_ANGLE, MAX_CORRECTION_ANGLE);
    pitch = constrain_float(pitch, -MAX_CORRECTION_ANGLE, MAX_CORRECTION_ANGLE);

    Quaternion q;
    q.from_euler(roll, pitch, 0.0f);
    q.normalize();

    return q;
}
