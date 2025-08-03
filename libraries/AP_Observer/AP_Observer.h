// #pragma once
// #include <AP_HAL/AP_HAL.h>
// #include <AP_Motors/AP_Motors.h>
// #include <GCS_MAVLink/GCS.h>  // MAVLink送信用
// #include "AP_Observer.h"
// #include <AP_InertialSensor/AP_InertialSensor.h>


// class AP_Observer {  // ✅ クラス名が完全一致しているか確認
// public:
//     void init() const;  // ✅ const修飾子を追加
//     void update() const; // ✅ const修飾子を追加

// private:
//     static uint32_t counter;

//     static constexpr float g = 9.7985f;
//     static constexpr float THRUST_SCALE = 6.3157f;
//     static constexpr float THRUST_OFFSET = -0.9995f;
//     static constexpr float UAV_mass = 1.4f;
// };


#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <Filter/LowPassFilter.h>

class AP_Observer {
public:
    void init() const;
    void update() const;
    
    // 外部アクセス用のゲッター関数
    Vector3f get_filtered_force() const { return current_filtered_force; }
    bool is_force_valid() const { return filter_initialized && (AP_HAL::millis() - last_update_ms < 100); }

private:
    static uint32_t counter;
    
    // フィルタ関連
    mutable LowPassFilterConstDtVector3f force_filter;
    mutable bool filter_initialized = false;
    mutable Vector3f current_filtered_force;     // 最新のフィルタ済み外力
    mutable uint32_t last_update_ms = 0;         // 最終更新時刻

    // フィルタパラメータ
    static constexpr float SAMPLE_FREQ = 100.0f;
    static constexpr float CUTOFF_FREQ = 5.0f;

    // 物理定数
    static constexpr float g = 9.7985f;
    static constexpr float THRUST_SCALE = 6.3157f;
    static constexpr float THRUST_OFFSET = -0.9995f;
    static constexpr float UAV_mass = 1.4f;
};