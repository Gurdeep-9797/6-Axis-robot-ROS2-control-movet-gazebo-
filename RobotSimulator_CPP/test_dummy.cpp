#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>
#include <iostream>

#include <cxxopts.hpp>
#include "engine/dx12_core.h"
#include "ui/ui_context.h"
#include "robot/robot_model.h"
#include "simulation/motion_engine.h"
#include "engine/ros_client.h"
#include <imgui.h>
#include <imgui_impl_dx12.h>
#include <imgui_impl_win32.h>
#include <memory>
#include <stdexcept>

#include "engine/camera.h"
#include "engine/scene_renderer.h"

// Forward declare ImGui handler
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Window Procedure
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hwnd, uMsg, wParam, lParam))
        return true;

    switch (uMsg) {
        case WM_DESTROY:
            std::cout << "WM_DESTROY received!" << std::endl;
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

    // Global singletons for this simple example
std::unique_ptr<DX12Core> g_dx12;
std::unique_ptr<UIContext> g_ui;
std::unique_ptr<RobotModel> g_robot;
std::unique_ptr<MotionEngine> g_motion;
std::unique_ptr<RosClient> g_ros;
std::unique_ptr<Camera> g_camera;
std::unique_ptr<SceneRenderer> g_scene;

int main(int argc, char** argv) {
    std::cout << "main() Started!" << std::endl;

    std::vector<std::string> args;
    for(int i = 0; i < argc; ++i) {
        args.push_back(argv[i]);
    }
    
    // Default Configuration
    HINSTANCE hInstance = GetModuleHandle(NULL);

    // Default Configuration
    std::string ros_uri = "ws://localhost:9090";
    std::string ref_topic = "";
    std::string mode_str = "sim";
    int winWidth = 1280;
    int winHeight = 800;

    // Parse CLI
    try {
        cxxopts::Options options("RobotSimulator", "6-Axis Robot Simulator");
        options.add_options()
            ("m,mode", "Control Mode (sim, real)", cxxopts::value<std::string>()->default_value("sim"))
            ("u,ros_uri", "ROS Websocket URI", cxxopts::value<std::string>()->default_value("ws://localhost:9090"))
            ("r,ref_topic", "Reference Joint Topic (e.g. /gazebo/joint_states)", cxxopts::value<std::string>()->default_value(""))
            ("w,width", "Window Width", cxxopts::value<int>()->default_value("1280"))
            ("h,height", "Window Height", cxxopts::value<int>()->default_value("800"))
            ("help", "Print help");

        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            MessageBoxA(NULL, options.help().c_str(), "Usage", MB_OK);
            return 0;
        }

        mode_str = result["mode"].as<std::string>();
        ros_uri = result["ros_uri"].as<std::string>();
        ref_topic = result["ref_topic"].as<std::string>();
        winWidth = result["width"].as<int>();
        winHeight = result["height"].as<int>();

    } catch (const std::exception& e) {
        MessageBoxA(NULL, e.what(), "CLI Parse Error", MB_OK | MB_ICONERROR);
        return 1;
    }

    // Register the window class.
    const wchar_t CLASS_NAME[] = L"RobotSimulator Window Class";

    WNDCLASS wc = { };
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = CLASS_NAME;

    RegisterClass(&wc);

    // Create the window.
    HWND hwnd = CreateWindowExW(
        0,                              // Optional window styles.
        CLASS_NAME,                     // Window class
        L"Robot Simulator C++20/DX12",  // Window text
        WS_OVERLAPPEDWINDOW,            // Window style
        CW_USEDEFAULT, CW_USEDEFAULT, winWidth, winHeight,
        NULL,       // Parent window    
        NULL,       // Menu
        hInstance,  // Instance handle
        NULL        // Additional application data
    );

    if (hwnd == NULL) {
        std::string err = "CreateWindowExW failed with Error Code: " + std::to_string(GetLastError());
        MessageBoxA(NULL, err.c_str(), "Window Creation Error", MB_OK | MB_ICONERROR);
        return 0;
    }

    try {
        RECT rect;
        GetClientRect(hwnd, &rect);
        int width = rect.right - rect.left;
        int height = rect.bottom - rect.top;
        
        std::cout << "Init DX12" << std::endl;
        g_dx12 = std::make_unique<DX12Core>();
        if (!g_dx12->Initialize(hwnd, width, height)) return 1;

        std::cout << "Init Robot" << std::endl;
        g_robot = std::make_unique<RobotModel>();

        std::cout << "Init Camera" << std::endl;
        g_camera = std::make_unique<Camera>((float)width / (float)height);
        
        std::cout << "Init ROS" << std::endl;
        // Initialize ROS Client and auto-connect
        g_ros = std::make_unique<RosClient>(ros_uri);
        g_ros->Connect();
        
        if (!ref_topic.empty()) {
            g_ros->SubscribeReference(ref_topic);
        }
        
        std::cout << "Init Motion Engine" << std::endl;
        g_motion = std::make_unique<MotionEngine>(g_robot.get(), g_ros.get());
        
        // Apply Mode
        if (mode_str == "real") {
            g_motion->SetMode(MotionEngine::ControlMode::RealHardware);
        } else {
            g_motion->SetMode(MotionEngine::ControlMode::Simulation);
        }

        std::cout << "Load URDF" << std::endl;
        // Load default robot model
        std::string urdfPath = args.empty() ? "" : args[0];
        auto lastSlash = urdfPath.find_last_of("\\/");
        if (lastSlash != std::string::npos) {
            urdfPath = urdfPath.substr(0, lastSlash) + "/assets/robot.urdf";
        } else {
            urdfPath = "assets/robot.urdf";
        }
        g_robot->LoadURDF(urdfPath);
        
        std::cout << "Init Scene" << std::endl;
        // Build 3D graphical scene
        g_scene = std::make_unique<SceneRenderer>(g_dx12->GetDevice());
        g_scene->BuildFromRobot(g_robot.get(), g_dx12->GetCommandList());
        
        std::cout << "Init UI" << std::endl;
        g_ui = std::make_unique<UIContext>(hwnd, g_dx12.get(), g_motion.get(), g_robot.get(), g_scene.get());
        std::cout << "All Subsystems Init OK" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "DX12 Init Error: " << e.what() << std::endl;
        return 1;
    }

    ShowWindow(hwnd, SW_SHOWDEFAULT);

    return 0;
}
