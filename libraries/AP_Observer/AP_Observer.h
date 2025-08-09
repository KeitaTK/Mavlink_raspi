// AP_Observer.h
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Math/AP_Math.h>

class AP_Observer {
public:
    void init() const;
    void update() const;

    // 補正クオータニオン取得用のゲッター関数
    Quaternion get_correction_quaternion() const { return current_correction_quat; }
    bool is_correction_valid() const { return (AP_HAL::millis() - last_update_ms < 100); }

private:
    static uint32_t counter;

    // 状態管理
    mutable Vector3f current_filtered_force;     // 最新の外力
    mutable Quaternion current_correction_quat;   // 最新の補正クオータニオン
    mutable uint32_t last_update_ms = 0;          // 最終更新時刻

    // 補正計算用の内部関数
    Quaternion calculate_correction_from_force(const Vector3f& force) const;

    // 補正パラメータ
    static constexpr float CORRECTION_GAIN = 0.3f;      // 補正の強さ
    static constexpr float FORCE_THRESHOLD = 0.5f;     // 最小外力 [N]
    static constexpr float MAX_CORRECTION_ANGLE = 0.1f; // 最大補正角度 [rad]

    // 物理定数
    static constexpr float g = 9.7985f;
    static constexpr float THRUST_SCALE = 6.3157f;
    static constexpr float THRUST_OFFSET = -0.9995f;
    static constexpr float UAV_mass = 1.4f;
};

extern AP_Observer ap_observer;
