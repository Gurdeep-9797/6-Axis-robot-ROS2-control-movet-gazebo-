#pragma once

#include <string>
#include <vector>
#include <functional>
#include <glm/glm.hpp>

// ─────────────────────────────────────────────────────────
// Program Instruction Types — maps to FR-HMI command palette
// ─────────────────────────────────────────────────────────
enum class InstructionType {
    // Logic Commands
    WaitMs,
    SetDO,
    While,
    IfElse,
    Goto,
    Wait,
    Pause,
    DoFile,
    DoFileEnd,
    Var,
    End,

    // Motion Commands
    PTP,        // Point-to-Point (joint interpolation)
    LIN,        // Linear (cartesian straight line)
    ARC,        // Arc motion (3-point arc)
    Circle,     // Full circle
    Spiral,     // Spiral path
    NSpiral,    // N-Spiral
    HSpiral,    // H-Spiral
    Spline,     // Cubic spline through waypoints
    NSpline,    // N-Spline
    Weave,      // Weave pattern (welding)
};

// ─────────────────────────────────────────────────────────
// A single program instruction in the work tree
// ─────────────────────────────────────────────────────────
struct ProgramInstruction {
    int lineNum = 0;
    InstructionType type = InstructionType::End;
    std::string displayText;       // e.g. "Lin(l1,60,0,0,0)"
    std::string comment;

    // Motion parameters
    std::string pointName;         // Named point reference (e.g. "l1", "p5")
    std::vector<float> jointTarget = {0,0,0,0,0,0};
    glm::vec3 cartesianTarget = glm::vec3(0);
    float speed = 100.0f;          // mm/s or %
    float acceleration = 180.0f;   // deg/s^2
    float blendRadius = 0.0f;      // mm - smooth transition
    int smoothing = 0;             // 0 = sharp, >0 = smooth corner

    // Logic parameters
    float waitTimeMs = 0.0f;
    int digitalOutput = 0;
    int doValue = 0;
    std::string filePath;
    std::string condition;

    // Arc-specific: via point
    glm::vec3 arcViaPoint = glm::vec3(0);

    // State
    bool isBreakpoint = false;
    bool isExecuting = false;
};

// ─────────────────────────────────────────────────────────
// Named teaching point (saved robot position)
// ─────────────────────────────────────────────────────────
struct TeachPoint {
    std::string name;
    std::string prefix;
    std::vector<float> joints = {0,0,0,0,0,0};
    glm::vec3 tcpPosition = glm::vec3(0);
    glm::vec3 tcpOrientation = glm::vec3(0); // RX, RY, RZ (Euler)
    int coordinateFrame = 0;                  // 0=Joint, 1=Base, 2=Tool, 3=Wobj
};

// ─────────────────────────────────────────────────────────
// Digital I/O state
// ─────────────────────────────────────────────────────────
struct DigitalIO {
    bool outputs[6] = {false};   // D00..D05
    bool inputs[6] = {false};
};

// ─────────────────────────────────────────────────────────
// Program Model — the full work tree / program
// ─────────────────────────────────────────────────────────
class ProgramModel {
public:
    ProgramModel();
    ~ProgramModel() = default;

    // Program management
    void NewProgram(const std::string& name);
    void Clear();

    // Instruction manipulation
    void AddInstruction(const ProgramInstruction& instr);
    void InsertInstruction(int index, const ProgramInstruction& instr);
    void RemoveInstruction(int index);
    void MoveInstruction(int fromIdx, int toIdx);

    // Quick-add helpers (from command palette clicks)
    void AddPTP(const std::string& pointName, float speed);
    void AddLIN(const std::string& pointName, float speed, float blend);
    void AddARC(const std::string& pointName, const std::string& viaPoint, float speed);
    void AddCircle(const std::string& pointName, float speed);
    void AddSpiral(const std::string& pointName, float speed);
    void AddWait(float ms);
    void AddSetDO(int pin, int value);
    void AddDoFile(const std::string& path);
    void AddDoFileEnd();
    void AddEnd();

    // Execution
    void Start();
    void Stop();
    void Pause();
    void StepForward();
    void StepBackward();
    bool IsRunning() const { return m_isRunning; }
    bool IsPaused() const { return m_isPaused; }
    int GetCurrentLine() const { return m_currentLine; }

    // Point management
    void AddTeachPoint(const TeachPoint& point);
    void RemoveTeachPoint(const std::string& name);
    const std::vector<TeachPoint>& GetTeachPoints() const { return m_teachPoints; }
    const TeachPoint* FindPoint(const std::string& name) const;

    // I/O
    DigitalIO& GetIO() { return m_io; }
    const DigitalIO& GetIO() const { return m_io; }

    // Accessors
    const std::string& GetFilename() const { return m_filename; }
    const std::vector<ProgramInstruction>& GetInstructions() const { return m_instructions; }
    std::vector<ProgramInstruction>& GetInstructionsMutable() { return m_instructions; }

    // Execution callback — called each step with the current instruction
    using ExecutionCallback = std::function<void(const ProgramInstruction&)>;
    void SetExecutionCallback(ExecutionCallback cb) { m_execCallback = cb; }

    // Update (called each frame while running)
    void Update(float deltaTime);

private:
    std::string m_filename = "program.lua";
    std::vector<ProgramInstruction> m_instructions;
    std::vector<TeachPoint> m_teachPoints;
    DigitalIO m_io;

    bool m_isRunning = false;
    bool m_isPaused = false;
    int m_currentLine = 0;
    float m_stepTimer = 0.0f;
    float m_stepDelay = 0.5f;   // seconds between auto-steps

    ExecutionCallback m_execCallback;

    void Renumber();
};
