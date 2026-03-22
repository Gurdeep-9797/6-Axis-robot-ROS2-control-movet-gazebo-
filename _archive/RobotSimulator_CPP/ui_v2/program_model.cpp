#include "program_model.h"
#include <algorithm>
#include <sstream>

ProgramModel::ProgramModel() {
    NewProgram("program.lua");
}

void ProgramModel::NewProgram(const std::string& name) {
    m_filename = name;
    m_instructions.clear();
    m_currentLine = 0;
    m_isRunning = false;
    m_isPaused = false;

    // Add default end instruction
    ProgramInstruction endInstr;
    endInstr.type = InstructionType::End;
    endInstr.displayText = "end";
    AddInstruction(endInstr);
}

void ProgramModel::Clear() {
    m_instructions.clear();
    m_currentLine = 0;
    m_isRunning = false;
    m_isPaused = false;
}

void ProgramModel::Renumber() {
    for (int i = 0; i < (int)m_instructions.size(); i++) {
        m_instructions[i].lineNum = i + 1;
    }
}

void ProgramModel::AddInstruction(const ProgramInstruction& instr) {
    // Insert before 'end' if one exists
    if (!m_instructions.empty() && m_instructions.back().type == InstructionType::End) {
        m_instructions.insert(m_instructions.end() - 1, instr);
    } else {
        m_instructions.push_back(instr);
    }
    Renumber();
}

void ProgramModel::InsertInstruction(int index, const ProgramInstruction& instr) {
    if (index < 0) index = 0;
    if (index > (int)m_instructions.size()) index = (int)m_instructions.size();
    m_instructions.insert(m_instructions.begin() + index, instr);
    Renumber();
}

void ProgramModel::RemoveInstruction(int index) {
    if (index >= 0 && index < (int)m_instructions.size()) {
        m_instructions.erase(m_instructions.begin() + index);
        Renumber();
    }
}

void ProgramModel::MoveInstruction(int fromIdx, int toIdx) {
    if (fromIdx < 0 || fromIdx >= (int)m_instructions.size()) return;
    if (toIdx < 0 || toIdx >= (int)m_instructions.size()) return;
    auto instr = m_instructions[fromIdx];
    m_instructions.erase(m_instructions.begin() + fromIdx);
    m_instructions.insert(m_instructions.begin() + toIdx, instr);
    Renumber();
}

// ─── Quick-add helpers ──────────────────────────────────

void ProgramModel::AddPTP(const std::string& pointName, float speed) {
    ProgramInstruction instr;
    instr.type = InstructionType::PTP;
    instr.pointName = pointName;
    instr.speed = speed;
    std::ostringstream oss;
    oss << "PTP(" << pointName << "," << speed << ",0,0)";
    instr.displayText = oss.str();
    AddInstruction(instr);
}

void ProgramModel::AddLIN(const std::string& pointName, float speed, float blend) {
    ProgramInstruction instr;
    instr.type = InstructionType::LIN;
    instr.pointName = pointName;
    instr.speed = speed;
    instr.blendRadius = blend;
    std::ostringstream oss;
    oss << "Lin(" << pointName << "," << speed << ",-1,0,0)";
    instr.displayText = oss.str();
    AddInstruction(instr);
}

void ProgramModel::AddARC(const std::string& pointName, const std::string& viaPoint, float speed) {
    ProgramInstruction instr;
    instr.type = InstructionType::ARC;
    instr.pointName = pointName;
    instr.speed = speed;
    std::ostringstream oss;
    oss << "Arc(" << viaPoint << "," << pointName << "," << speed << ",0,0)";
    instr.displayText = oss.str();
    AddInstruction(instr);
}

void ProgramModel::AddCircle(const std::string& pointName, float speed) {
    ProgramInstruction instr;
    instr.type = InstructionType::Circle;
    instr.pointName = pointName;
    instr.speed = speed;
    std::ostringstream oss;
    oss << "Circle(" << pointName << "," << speed << ",0)";
    instr.displayText = oss.str();
    AddInstruction(instr);
}

void ProgramModel::AddSpiral(const std::string& pointName, float speed) {
    ProgramInstruction instr;
    instr.type = InstructionType::Spiral;
    instr.pointName = pointName;
    instr.speed = speed;
    std::ostringstream oss;
    oss << "Spiral(" << pointName << "," << speed << ")";
    instr.displayText = oss.str();
    AddInstruction(instr);
}

void ProgramModel::AddWait(float ms) {
    ProgramInstruction instr;
    instr.type = InstructionType::WaitMs;
    instr.waitTimeMs = ms;
    std::ostringstream oss;
    oss << "WaitMs(" << (int)ms << ")";
    instr.displayText = oss.str();
    AddInstruction(instr);
}

void ProgramModel::AddSetDO(int pin, int value) {
    ProgramInstruction instr;
    instr.type = InstructionType::SetDO;
    instr.digitalOutput = pin;
    instr.doValue = value;
    std::ostringstream oss;
    oss << "SetDO(" << pin << "," << value << ",0,0)";
    instr.displayText = oss.str();
    AddInstruction(instr);
}

void ProgramModel::AddDoFile(const std::string& path) {
    ProgramInstruction instr;
    instr.type = InstructionType::DoFile;
    instr.filePath = path;
    std::ostringstream oss;
    oss << "NewDoFile(\"" << path << "\"...)";
    instr.displayText = oss.str();
    AddInstruction(instr);
}

void ProgramModel::AddDoFileEnd() {
    ProgramInstruction instr;
    instr.type = InstructionType::DoFileEnd;
    instr.displayText = "DoFileEnd()";
    AddInstruction(instr);
}

void ProgramModel::AddEnd() {
    ProgramInstruction instr;
    instr.type = InstructionType::End;
    instr.displayText = "end";
    m_instructions.push_back(instr);
    Renumber();
}

// ─── Teach Points ───────────────────────────────────────

void ProgramModel::AddTeachPoint(const TeachPoint& point) {
    // Replace if exists
    for (auto& tp : m_teachPoints) {
        if (tp.name == point.name) {
            tp = point;
            return;
        }
    }
    m_teachPoints.push_back(point);
}

void ProgramModel::RemoveTeachPoint(const std::string& name) {
    m_teachPoints.erase(
        std::remove_if(m_teachPoints.begin(), m_teachPoints.end(),
            [&](const TeachPoint& tp) { return tp.name == name; }),
        m_teachPoints.end());
}

const TeachPoint* ProgramModel::FindPoint(const std::string& name) const {
    for (const auto& tp : m_teachPoints) {
        if (tp.name == name) return &tp;
    }
    return nullptr;
}

// ─── Execution ──────────────────────────────────────────

void ProgramModel::Start() {
    if (m_instructions.empty()) return;
    m_isRunning = true;
    m_isPaused = false;
    m_currentLine = 0;
    m_stepTimer = 0.0f;

    // Mark first instruction
    for (auto& instr : m_instructions) instr.isExecuting = false;
    if (!m_instructions.empty()) m_instructions[0].isExecuting = true;
}

void ProgramModel::Stop() {
    m_isRunning = false;
    m_isPaused = false;
    for (auto& instr : m_instructions) instr.isExecuting = false;
}

void ProgramModel::Pause() {
    m_isPaused = !m_isPaused;
}

void ProgramModel::StepForward() {
    if (m_currentLine < (int)m_instructions.size() - 1) {
        m_instructions[m_currentLine].isExecuting = false;
        m_currentLine++;
        m_instructions[m_currentLine].isExecuting = true;
        if (m_execCallback) {
            m_execCallback(m_instructions[m_currentLine]);
        }
    } else {
        Stop();
    }
}

void ProgramModel::StepBackward() {
    if (m_currentLine > 0) {
        m_instructions[m_currentLine].isExecuting = false;
        m_currentLine--;
        m_instructions[m_currentLine].isExecuting = true;
    }
}

void ProgramModel::Update(float deltaTime) {
    if (!m_isRunning || m_isPaused) return;
    if (m_currentLine >= (int)m_instructions.size()) {
        Stop();
        return;
    }

    m_stepTimer += deltaTime;
    if (m_stepTimer >= m_stepDelay) {
        m_stepTimer = 0.0f;

        // Execute current
        if (m_execCallback) {
            m_execCallback(m_instructions[m_currentLine]);
        }

        // Handle wait instructions
        auto& curr = m_instructions[m_currentLine];
        if (curr.type == InstructionType::WaitMs) {
            m_stepDelay = curr.waitTimeMs / 1000.0f;
        } else {
            m_stepDelay = 0.5f; // default step speed
        }

        // Advance
        StepForward();
    }
}
