#pragma once

#include <imgui.h>
#include "../simulation/motion_engine.h"

class CalibrationPanel {
public:
    CalibrationPanel(MotionEngine* motion);
    ~CalibrationPanel();

    void Render();

private:
    MotionEngine* m_motion;
};
