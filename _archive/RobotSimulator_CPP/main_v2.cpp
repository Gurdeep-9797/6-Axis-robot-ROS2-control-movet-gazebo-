#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>
#include <iostream>
#include <cxxopts.hpp>
#include "engine/dx12_core.h"
#include "ui_v2/teach_pendant_ui.h"
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

// Global singletons
std::unique_ptr<DX12Core> g_dx12;
std::unique_ptr<TeachPendantUI> g_ui;
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
bool g_initialized = false;  // Guard: WndProc must not touch ImGui until init done

// Forward declare ImGui handler
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Window Procedure
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    // Only forward to ImGui AFTER initialization
    if (g_initialized) {
        if (ImGui_ImplWin32_WndProcHandler(hwnd, uMsg, wParam, lParam))
            return true;
    }

    switch (uMsg) {
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        case WM_SIZE:
             if (g_initialized && g_dx12) {
                 RECT rect;
                 GetClientRect(hwnd, &rect);
                 int w = rect.right - rect.left;
                 int h = rect.bottom - rect.top;
                 if (w > 0 && h > 0) {
                     g_dx12->Resize(w, h);
                     if (g_camera) g_camera->SetAspect((float)w / (float)h);
                 }
             }
             return 0;
             
        case WM_LBUTTONDOWN:
            if (g_initialized && !ImGui::GetIO().WantCaptureMouse) {
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
            if (g_initialized && !ImGui::GetIO().WantCaptureMouse) {
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
            if (g_initialized && g_camera) {
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
            if (g_initialized && !ImGui::GetIO().WantCaptureMouse && g_camera) {
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
    std::vector<std::string> args;
    for(int i = 0; i < argc; ++i) {
        args.push_back(argv[i]);
    }
    
    HINSTANCE hInstance = GetModuleHandle(NULL);

    // Default Configuration
    std::string ros_uri = "ws://localhost:9090";
    std::string ref_topic = "";
    std::string mode_str = "sim";
    int winWidth = 1280;
    int winHeight = 800;

    // Parse CLI
    try {
        cxxopts::Options options("TeachPendant_V2", "FR-HMI Teach Pendant V2 — 6-Axis Robot Controller");
        options.add_options()
            ("m,mode", "Control Mode (sim, real)", cxxopts::value<std::string>()->default_value("sim"))
            ("u,ros_uri", "ROS Websocket URI", cxxopts::value<std::string>()->default_value("ws://localhost:9090"))
            ("r,ref_topic", "Reference Joint Topic", cxxopts::value<std::string>()->default_value(""))
            ("w,width", "Window Width", cxxopts::value<int>()->default_value("1280"))
            ("h,height", "Window Height", cxxopts::value<int>()->default_value("800"))
            ("help", "Print help");

        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            std::cout << options.help() << std::endl;
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
    const wchar_t CLASS_NAME[] = L"TeachPendant_V2 Window Class";

    WNDCLASS wc = { };
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = CLASS_NAME;

    RegisterClass(&wc);

    // Create the window.
    HWND hwnd = CreateWindowExW(
        0,
        CLASS_NAME,
        L"FR-HMI TeachPendant V2 | 6-Axis Robot Controller",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, winWidth, winHeight,
        NULL, NULL, hInstance, NULL
    );

    if (hwnd == NULL) {
        std::string err = "CreateWindowExW failed: " + std::to_string(GetLastError());
        MessageBoxA(NULL, err.c_str(), "Window Creation Error", MB_OK | MB_ICONERROR);
        return 0;
    }

    try {
        RECT rect;
        GetClientRect(hwnd, &rect);
        int width = rect.right - rect.left;
        int height = rect.bottom - rect.top;
        
        // Initialize DX12 (defensive init)
        g_dx12 = std::make_unique<DX12Core>();
        if (!g_dx12->Initialize(hwnd, width, height)) {
            return 1;
        }

        g_robot = std::make_unique<RobotModel>();
        g_camera = std::make_unique<Camera>((float)width / (float)height);
        
        // ROS Client
        g_ros = std::make_unique<RosClient>(ros_uri);
        g_ros->Connect();
        if (!ref_topic.empty()) {
            g_ros->SubscribeReference(ref_topic);
        }
        
        g_motion = std::make_unique<MotionEngine>(g_robot.get(), g_ros.get());
        
        if (mode_str == "real") {
            g_motion->SetMode(MotionEngine::ControlMode::RealHardware);
        } else {
            g_motion->SetMode(MotionEngine::ControlMode::Simulation);
        }

        // Load robot model
        std::string urdfPath = args.empty() ? "" : args[0];
        auto lastSlash = urdfPath.find_last_of("\\/");
        if (lastSlash != std::string::npos) {
            urdfPath = urdfPath.substr(0, lastSlash) + "/assets/robot.urdf";
        } else {
            urdfPath = "assets/robot.urdf";
        }
        /*
        g_robot->LoadURDF(urdfPath);
        std::cout << "V2: Robot loaded, building scene..." << std::endl;
        
        // Build 3D scene — must open command list for GPU buffer uploads
        g_scene = std::make_unique<SceneRenderer>(g_dx12->GetDevice());
        
        // Reset command list to recording state for mesh uploads
        g_dx12->GetCommandAllocator()->Reset();
        g_dx12->GetCommandList()->Reset(g_dx12->GetCommandAllocator(), nullptr);
        
        std::cout << "V2: BuildFromRobot..." << std::endl;
        g_scene->BuildFromRobot(g_robot.get(), g_dx12->GetCommandList());
        
        std::cout << "V2: FlushCommandList..." << std::endl;
        // Execute and wait for GPU uploads to complete
        g_dx12->FlushCommandList();
        */
        
        std::cout << "V2: Creating TeachPendantUI..." << std::endl;
        // ── V2 Teach Pendant UI ──
        g_ui = std::make_unique<TeachPendantUI>(hwnd, g_dx12.get(), g_motion.get(), nullptr, nullptr);
        
        std::cout << "V2: Init complete!" << std::endl;
    } catch (const std::exception& e) {
        MessageBoxA(NULL, e.what(), "Init Error", MB_OK | MB_ICONERROR);
        std::cerr << "Init Error: " << e.what() << std::endl;
        return 1;
    }

    ShowWindow(hwnd, SW_SHOWDEFAULT);
    g_initialized = true;

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
            if (g_dx12 && g_ui) {
                try {
                    std::cout << "V2: Frame Start" << std::endl;
                    g_motion->Update(0.016f);
                    
                    std::cout << "V2: Before BeginFrame - DX12" << std::endl;
                    ImGui_ImplDX12_NewFrame();
                    std::cout << "V2: Before BeginFrame - Win32" << std::endl;
                    ImGui_ImplWin32_NewFrame();
                    std::cout << "V2: Before BeginFrame - ImGui" << std::endl;
                    ImGui::NewFrame();
                    
                    std::cout << "V2: Before Render" << std::endl;
                    ImGui::Begin("Test Output");
                    ImGui::Text("DX12 is working perfectly!");
                    ImGui::End();
                    
                    std::cout << "V2: Before EndFrame" << std::endl;
                    ImGui::Render();
                    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
                        ImGui::UpdatePlatformWindows();
                        ImGui::RenderPlatformWindowsDefault(NULL, (void*)g_dx12->GetCommandList());
                    }

                    std::cout << "V2: Before DX12 Render" << std::endl;
                    g_dx12->Render([&](ID3D12GraphicsCommandList* cmdList) {
                        // Update Constant Buffer
                        if (g_dx12->GetCbvDataBegin() && g_camera) {
                            SceneConstantBuffer cb = {};
                            cb.model = glm::mat4(1.0f);
                            cb.view = g_camera->GetViewMatrix();
                            cb.projection = g_camera->GetProjectionMatrix();
                            cb.ambientColor = glm::vec4(0.2f, 0.2f, 0.25f, 1.0f);
                            cb.directionalLightDir = glm::vec4(-1.0f, -2.0f, -1.5f, 0.0f);
                            cb.directionalLightColor = glm::vec4(0.8f, 0.8f, 0.8f, 1.0f);
                            memcpy(g_dx12->GetCbvDataBegin(), &cb, sizeof(SceneConstantBuffer));
                        }
                        
                        // Draw 3D Scene
                        if (g_scene) {
                            g_scene->Draw(cmdList, g_robot.get(), nullptr, g_dx12->GetCbvDataBegin());
                        }

                        // ImGui draw
                        ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), cmdList);
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
