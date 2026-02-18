#include <iostream>
#include <vector>
#include <cmath>
#include <string>

// Minimal test framework
static int g_passed = 0;
static int g_failed = 0;

#define TEST_CASE(name) \
    void test_##name(); \
    struct Register_##name { Register_##name() { test_##name(); } } reg_##name; \
    void test_##name()

#define CHECK(cond) do { \
    if (!(cond)) { \
        std::cerr << "  FAIL: " << #cond << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
        g_failed++; return; \
    } \
} while(0)

#define CHECK_CLOSE(a, b, eps) do { \
    if (std::abs((a)-(b)) > (eps)) { \
        std::cerr << "  FAIL: " << (a) << " != " << (b) << " (eps=" << (eps) << ") at " << __FILE__ << ":" << __LINE__ << std::endl; \
        g_failed++; return; \
    } \
} while(0)

#define PASS() g_passed++

// Include only non-DX12 headers for headless testing
#include "../robot/robot_model.h"

// ─────────────────────────────────────────────────────────
// Test: ParseVec3-style math  
// ─────────────────────────────────────────────────────────
TEST_CASE(MathUtils) {
    std::cout << "[TEST] MathUtils..." << std::endl;
    
    // Linear interpolation
    float start = 0.0f, end = 10.0f, t = 0.5f;
    float res = start + (end - start) * t;
    CHECK_CLOSE(res, 5.0f, 0.001f);
    
    // GLM basics
    glm::vec3 v(1.0f, 2.0f, 3.0f);
    CHECK_CLOSE(v.x, 1.0f, 0.001f);
    CHECK_CLOSE(v.y, 2.0f, 0.001f);
    CHECK_CLOSE(v.z, 3.0f, 0.001f);
    
    // Identity matrix
    glm::mat4 I(1.0f);
    CHECK_CLOSE(I[0][0], 1.0f, 0.001f);
    CHECK_CLOSE(I[3][3], 1.0f, 0.001f);
    
    std::cout << "  PASSED" << std::endl;
    PASS();
}

// ─────────────────────────────────────────────────────────
// Test: RobotModel construction  
// ─────────────────────────────────────────────────────────
TEST_CASE(RobotModelInit) {
    std::cout << "[TEST] RobotModelInit..." << std::endl;
    
    RobotModel model;
    CHECK(model.GetLinks().empty());
    CHECK(model.GetJoints().empty());
    CHECK(model.GetDOF() == 0);
    
    std::cout << "  PASSED" << std::endl;
    PASS();
}

// ─────────────────────────────────────────────────────────
// Test: Supported formats list
// ─────────────────────────────────────────────────────────
TEST_CASE(SupportedFormats) {
    std::cout << "[TEST] SupportedFormats..." << std::endl;
    
    auto& fmts = RobotModel::GetSupportedFormats();
    CHECK(fmts.size() >= 6); // STL, OBJ, DAE, FBX, STEP, 3DS at minimum
    
    // Check STL is supported
    bool hasSTL = false;
    for (auto& f : fmts) {
        if (f == ".stl") hasSTL = true;
    }
    CHECK(hasSTL);
    
    std::cout << "  PASSED" << std::endl;
    PASS();
}

// ─────────────────────────────────────────────────────────
// Test: GLM transforms (FK math foundation)
// ─────────────────────────────────────────────────────────
TEST_CASE(TransformMath) {
    std::cout << "[TEST] TransformMath..." << std::endl;
    
    // Translation
    glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(1, 2, 3));
    CHECK_CLOSE(T[3][0], 1.0f, 0.001f);
    CHECK_CLOSE(T[3][1], 2.0f, 0.001f);
    CHECK_CLOSE(T[3][2], 3.0f, 0.001f);
    
    // Rotation around Z by 90 degrees
    glm::mat4 R = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0, 0, 1));
    // After rotating (1,0,0) by 90° around Z, we should get (0,1,0)
    glm::vec4 v = R * glm::vec4(1, 0, 0, 1);
    CHECK_CLOSE(v.x, 0.0f, 0.001f);
    CHECK_CLOSE(v.y, 1.0f, 0.001f);
    
    std::cout << "  PASSED" << std::endl;
    PASS();
}

int main() {
    std::cout << "═══════════════════════════════════════════" << std::endl;
    std::cout << "  Robot Simulator — Component Tests" << std::endl;
    std::cout << "═══════════════════════════════════════════" << std::endl;
    
    // Tests already ran via static registration above.
    
    std::cout << "───────────────────────────────────────────" << std::endl;
    std::cout << "  Results: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    std::cout << "═══════════════════════════════════════════" << std::endl;
    
    return g_failed > 0 ? 1 : 0;
}
