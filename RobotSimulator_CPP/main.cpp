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

// Global singletons for this simple example
std::unique_ptr<DX12Core> g_dx12;
std::unique_ptr<UIContext> g_ui;
std::unique_ptr<RobotModel> g_robot;
std::unique_ptr<MotionEngine> g_motion;
std::unique_ptr<RosClient> g_ros;
std::unique_ptr<Camera> g_camera;
std::unique_ptr<SceneRenderer> g_scene;

// Mouse tracking
bool g_isDragging = false;
bool g_isPanning = false;
int g_lastMouseX = 0;
int g_lastMouseY = 0;

// Forward declare ImGui handler
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Window Procedure
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hwnd, uMsg, wParam, lParam))
        return true;

    switch (uMsg) {
        case WM_DESTROY:
            MessageBoxA(hwnd, "WM_DESTROY received!", "Debug Trace", MB_OK);
            PostQuitMessage(0);
            return 0;
        case WM_SIZE:
             if (g_dx12) {
                 RECT rect;
                 GetClientRect(hwnd, &rect);
                 float w = (float)(rect.right - rect.left);
                 float h = (float)(rect.bottom - rect.top);
                 g_dx12->Resize((int)w, (int)h);
                 if (g_camera && h > 0) g_camera->SetAspect(w / h);
             }
             return 0;
             
        case WM_LBUTTONDOWN:
            if (!ImGui::GetIO().WantCaptureMouse) {
                g_isDragging = true;
                g_lastMouseX = LOWORD(lParam);
                g_lastMouseY = HIWORD(lParam);
                SetCapture(hwnd);
            }
            return 0;
            
        case WM_LBUTTONUP:
            g_isDragging = false;
            ReleaseCapture();
            return 0;
            
        case WM_MBUTTONDOWN:
            if (!ImGui::GetIO().WantCaptureMouse) {
                g_isPanning = true;
                g_lastMouseX = LOWORD(lParam);
                g_lastMouseY = HIWORD(lParam);
                SetCapture(hwnd);
            }
            return 0;
            
        case WM_MBUTTONUP:
            g_isPanning = false;
            ReleaseCapture();
            return 0;
            
        case WM_MOUSEMOVE:
            if (g_camera) {
                int x = LOWORD(lParam);
                int y = HIWORD(lParam);
                int dx = x - g_lastMouseX;
                int dy = y - g_lastMouseY;
                
                if (g_isDragging) {
                    g_camera->Orbit((float)dx, (float)dy);
                } else if (g_isPanning) {
                    g_camera->Pan((float)dx, (float)dy);
                }
                
                g_lastMouseX = x;
                g_lastMouseY = y;
            }
            return 0;
            
        case WM_MOUSEWHEEL:
            if (!ImGui::GetIO().WantCaptureMouse && g_camera) {
                float delta = GET_WHEEL_DELTA_WPARAM(wParam) / (float)WHEEL_DELTA;
                g_camera->Zoom(delta);
            }
            return 0;
        case WM_PAINT: {
            PAINTSTRUCT ps;
            BeginPaint(hwnd, &ps);
            EndPaint(hwnd, &ps);
            return 0;
        }
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

int main(int argc, char** argv) {
    MessageBoxA(NULL, "main() Started!", "Debug Trace", MB_OK);

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
        
        // Initialize Subsystems
        g_dx12 = std::make_unique<DX12Core>(hwnd, width, height);
        g_robot = std::make_unique<RobotModel>();
        g_camera = std::make_unique<Camera>((float)width / (float)height);
        
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

        // Load default robot model
        std::string urdfPath = args.empty() ? "" : args[0];
        auto lastSlash = urdfPath.find_last_of("\\/");
        if (lastSlash != std::string::npos) {
            urdfPath = urdfPath.substr(0, lastSlash) + "/assets/robot.urdf";
        } else {
            urdfPath = "assets/robot.urdf";
        }
        g_robot->LoadURDF(urdfPath);
        
        // Build 3D graphical scene
        g_scene = std::make_unique<SceneRenderer>(g_dx12->GetDevice());
        g_scene->BuildFromRobot(g_robot.get(), g_dx12->GetCommandList());
        
        g_ui = std::make_unique<UIContext>(hwnd, g_dx12.get(), g_motion.get(), g_robot.get(), g_scene.get());
        
    } catch (const std::exception& e) {
        MessageBoxA(NULL, e.what(), "DX12 Init Error", MB_OK | MB_ICONERROR);
        std::cerr << "DX12 Init Error: " << e.what() << std::endl;
        return 1;
    }

    ShowWindow(hwnd, SW_SHOWDEFAULT);

    MessageBoxA(NULL, "Entering message loop...", "Debug Trace", MB_OK);

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
                try {
                    // Update Motion
                    g_motion->Update(0.016f);
                    
                    // UI Frame
                    g_ui->BeginFrame();
                    g_ui->Render(); 
                    g_ui->EndFrame();

                    // Render DX12
                    g_dx12->Render([&](ID3D12GraphicsCommandList* cmdList) {
                        
                        // Update Constant Buffer
                        if (g_dx12->GetCbvDataBegin() && g_camera) {
                            SceneConstantBuffer cb = {};
                            cb.model = glm::mat4(1.0f); // Identity for background/grid
                            cb.view = g_camera->GetViewMatrix();
                            cb.projection = g_camera->GetProjectionMatrix();
                            cb.ambientColor = glm::vec4(0.2f, 0.2f, 0.25f, 1.0f);
                            cb.directionalLightDir = glm::vec4(-1.0f, -2.0f, -1.5f, 0.0f);
                            cb.directionalLightColor = glm::vec4(0.8f, 0.8f, 0.8f, 1.0f);
                            
                            memcpy(g_dx12->GetCbvDataBegin(), &cb, sizeof(SceneConstantBuffer));
                        }
                        
                        // Draw 3D Scene
                        if (g_scene) {
                            g_scene->Draw(cmdList, g_robot.get(), g_ui.get(), g_dx12->GetCbvDataBegin());
                        }

                        // Render ImGui draw data into the DX12 command list
                        ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), cmdList);
                        // Note: Viewport platform windows are handled in UIContext::EndFrame()
                    });
                } catch (const std::exception& e) {
                    std::string err = "Frame Error: " + std::string(e.what());
                    MessageBoxA(NULL, err.c_str(), "Fatal Runtime Error", MB_OK | MB_ICONERROR);
                    running = false;
                } catch (...) {
                    MessageBoxA(NULL, "Unknown exception in frame loop", "Fatal Runtime Error", MB_OK | MB_ICONERROR);
                    running = false;
                }
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
