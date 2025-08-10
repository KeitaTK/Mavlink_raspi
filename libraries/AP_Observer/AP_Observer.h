// AP_Observer.h
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS.h>

class AP_Observer {
public:
    void init() const;
    void update() const;

    // 補正クオータニオン取得用のゲッター
    Quaternion get_correction_quaternion() const { return current_correction_quat; }
    bool is_correction_valid() const { return (AP_HAL::millis() - last_update_ms < 100); }

    // パラメータ定義テーブル
    static const struct AP_Param::GroupInfo var_info[];

private:
    static uint32_t counter;

    // 状態管理
    mutable Vector3f    current_filtered_force;
    mutable Quaternion  current_correction_quat;
    mutable uint32_t    last_update_ms = 0;

    // 補正計算用
    Quaternion calculate_correction_from_force(const Vector3f& force) const;

    // パラメータ（MAVLinkで変更可能）
    AP_Float _correction_gain;

    // その他の定数
    static constexpr float FORCE_THRESHOLD     = 0.2f;
    static constexpr float MAX_CORRECTION_ANGLE = 0.5f;
    static constexpr float g                   = 9.7985f;
    static constexpr float THRUST_SCALE        = 6.3157f;
    static constexpr float THRUST_OFFSET       = -0.9995f;
    static constexpr float UAV_mass            = 1.4f;
};

extern AP_Observer ap_observer;
