#include "AP_MyModule.h"
#include <AP_HAL/AP_HAL.h>

void AP_MyModule::init() const {  // ✅ クラス名とconst修飾子が一致
    hal.console->printf("AP_MyModule initialized\n");
}

void AP_MyModule::update() const {  // ✅ クラス名とconst修飾子が一致
    // 実装内容
}