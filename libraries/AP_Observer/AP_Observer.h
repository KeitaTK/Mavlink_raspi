// #pragma once

// #include <AP_HAL/AP_HAL.h>
// #include <AP_Vehicle/AP_Vehicle.h>
// #include <AP_Motors/AP_Motors.h>
// #include <GCS_MAVLink/GCS.h>  // MAVLink送信用


// class AP_Observer {
// public:
//     void init();
//     void update();

// private:
//     const AP_HAL::HAL& hal = AP_HAL::get_HAL();
//     // uint32_t _last_ms = 0;
//     // static constexpr uint32_t PERIOD_MS = 50;  // 20 Hz → 1000 ms/20 = 50 ms
// };

#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS.h>  // MAVLink送信用

class AP_Observer {  // ✅ クラス名が完全一致しているか確認
public:
    void init() const;  // ✅ const修飾子を追加
    void update() const; // ✅ const修飾子を追加

private:
    int counter = 0;
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();  // ✅ const参照
};