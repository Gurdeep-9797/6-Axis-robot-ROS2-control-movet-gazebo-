#include <windows.h>
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

// Global singletons for this simple example
std::unique_ptr<DX12Core> g_dx12;
std::unique_ptr<UIContext> g_ui;
std::unique_ptr<RobotModel> g_robot;
std::unique_ptr<MotionEngine> g_motion;
std::unique_ptr<RosClient> g_ros;

// Forward declare ImGui handler
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Window Procedure
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hwnd, uMsg, wParam, lParam))
        return true;

    switch (uMsg) {
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        case WM_SIZE:
             if (g_dx12) {
                 RECT rect;
                 GetClientRect(hwnd, &rect);
                 g_dx12->Resize(rect.right - rect.left, rect.bottom - rect.top);
             }
             return 0;
        case WM_PAINT: {
            // Validate window
            return 0;
        }
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow) {
    // Convert Wide Args to UTF-8 for cxxopts
    int argc;
    LPWSTR* argvW = CommandLineToArgvW(GetCommandLineW(), &argc);
    std::vector<std::string> args;
    std::vector<char*> argvC;
    if (argvW) {
        for(int i=0; i<argc; ++i) {
            int len = WideCharToMultiByte(CP_UTF8, 0, argvW[i], -1, NULL, 0, NULL, NULL);
            if(len > 0) {
                 std::string s(len-1, '\0');
                 WideCharToMultiByte(CP_UTF8, 0, argvW[i], -1, &s[0], len, NULL, NULL);
                 args.push_back(s);
            }
        }
        LocalFree(argvW);
    }
    for(auto& s : args) argvC.push_back(&s[0]);
    char** argv = argvC.data();

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
    HWND hwnd = CreateWindowEx(
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
        return 0;
    }

    try {
        RECT rect;
        GetClientRect(hwnd, &rect);
        int width = rect.right - rect.left;
        int height = rect.bottom - rect.top;
        
        // Initialize Subsystems
        g_dx12 = std::make_unique<DX12Core>(hwnd, width, height);
        g_robot = std::make_unique<RobotModel>();
        
        // Initialize ROS Client and auto-connect
        g_ros = std::make_unique<RosClient>(ros_uri);
        g_ros->Connect();
        
        if (!ref_topic.empty()) {
            g_ros->SubscribeReference(ref_topic);
        }
        
        g_motion = std::make_unique<MotionEngine>(g_robot.get(), g_ros.get());
        
        // Apply Mode
        if (mode_str == "real") {
            g_motion->SetMode(MotionEngine::ControlMode::RealHardware);
        } else {
            g_motion->SetMode(MotionEngine::ControlMode::Simulation);
        }

        g_ui = std::make_unique<UIContext>(hwnd, g_dx12.get(), g_motion.get());
        
        // Load default robot if available
        // g_robot->LoadURDF("assets/robot.urdf");
        
    } catch (const std::exception& e) {
        MessageBoxA(NULL, e.what(), "DX12 Init Error", MB_OK | MB_ICONERROR);
        return 1;
    }

    ShowWindow(hwnd, nCmdShow);

    // Run the message loop.
    MSG msg = { };
    bool running = true;
    while (running) {
        if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                running = false;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        } else {
            // Frame Logic
            if (g_dx12 && g_ui) {
                
                // Update Motion
                g_motion->Update(0.016f);
                
                // UI Frame
                g_ui->BeginFrame();
                g_ui->Render(); 
                g_ui->EndFrame();

                // Render DX12
                g_dx12->Render([&](ID3D12GraphicsCommandList* cmdList) {
                    // Render ImGui draw data into the DX12 command list
                    ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), cmdList);
                    // Note: Viewport platform windows are handled in UIContext::EndFrame()
                });
            }
        }
    }
    
    // Cleanup
    g_ui.reset();
    g_motion.reset();
    g_ros.reset();
    g_robot.reset();
    g_dx12.reset();

    return 0;
}
