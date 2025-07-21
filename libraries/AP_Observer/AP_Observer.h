#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS.h>  // MAVLink送信用
#include "AP_Observer.h"


class AP_Observer {  // ✅ クラス名が完全一致しているか確認
public:
    void init() const;  // ✅ const修飾子を追加
    void update() const; // ✅ const修飾子を追加

private:
    static constexpr float THRUST_SCALE = 6.3157f;
    static constexpr float THRUST_OFFSET = -0.9995f;
    static constexpr float UAV_mass = -1.4f;
};