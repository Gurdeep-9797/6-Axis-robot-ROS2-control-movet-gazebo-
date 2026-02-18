#pragma once

#include "ui_context.h"
#include "../simulation/motion_engine.h"

class CalibrationPanel {
public:
    CalibrationPanel(MotionEngine* motion);
    ~CalibrationPanel();

    void Render();

private:
    MotionEngine* m_motion;
};
