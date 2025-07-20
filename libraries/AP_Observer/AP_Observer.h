#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS.h>  // MAVLink送信用


class AP_Observer {  // ✅ クラス名が完全一致しているか確認
public:
    void init() const;  // ✅ const修飾子を追加
    void update() const; // ✅ const修飾子を追加

private:
    static constexpr float THRUST_SCALE = 1.5935f;
    static constexpr float THRUST_OFFSET = -0.2589f; 
};