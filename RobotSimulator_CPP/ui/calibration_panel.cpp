#include "calibration_panel.h"
#include <string>

CalibrationPanel::CalibrationPanel(MotionEngine* motion) : m_motion(motion) {}
CalibrationPanel::~CalibrationPanel() {}

void CalibrationPanel::Render() {
    ImGui::Begin("Calibration & Setup");

    // Mode Selection
    MotionEngine::ControlMode currentMode = m_motion->GetMode();
    const char* modePreview = (currentMode == MotionEngine::ControlMode::Simulation) ? "Simulation (Gazebo)" : "Real Hardware (ESP32)";
    
    if (ImGui::BeginCombo("Operation Mode", modePreview)) {
        if (ImGui::Selectable("Simulation (Gazebo)", currentMode == MotionEngine::ControlMode::Simulation)) {
            m_motion->SetMode(MotionEngine::ControlMode::Simulation);
        }
        if (ImGui::Selectable("Real Hardware (ESP32)", currentMode == MotionEngine::ControlMode::RealHardware)) {
            m_motion->SetMode(MotionEngine::ControlMode::RealHardware);
        }
        ImGui::EndCombo();
    }
    
    ImGui::Separator();
    
    if (ImGui::CollapsingHeader("Encoder Calibration", ImGuiTreeNodeFlags_DefaultOpen)) {
        MotionEngine::CalibrationData& cal = m_motion->GetCalibration();
        
        for (int i = 0; i < 6; i++) {
            std::string label = "Joint " + std::to_string(i+1) + " Offset (rad)";
            ImGui::DragFloat(label.c_str(), &cal.encoderOffsets[i], 0.01f, -3.14f, 3.14f);
        }
    }

    if (ImGui::CollapsingHeader("Hardware Connection")) {
        MotionEngine::CalibrationData& cal = m_motion->GetCalibration();
        
        // Simple string input for now (could be COM port drop-down later)
        char buf[64];
        strcpy_s(buf, cal.connectionString.c_str());
        if (ImGui::InputText("Port / IP", buf, 64)) {
            cal.connectionString = std::string(buf);
        }
        
        if (ImGui::Button("Connect")) {
            // Trigger connection logic
        }
        ImGui::SameLine();
        if (ImGui::Button("Disconnect")) {
            // Trigger disconnect logic
        }
    }
    
    ImGui::End();
}
