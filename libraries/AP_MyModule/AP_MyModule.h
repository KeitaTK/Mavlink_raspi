#pragma once
#include <AP_HAL/AP_HAL.h>

class AP_MyModule {  // ✅ クラス名が完全一致しているか確認
public:
    void init() const;  // ✅ const修飾子を追加
    void update() const; // ✅ const修飾子を追加

private:
    int counter = 0;
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();  // ✅ const参照
};