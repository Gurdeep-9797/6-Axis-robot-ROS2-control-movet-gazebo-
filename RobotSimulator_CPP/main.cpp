#include <windows.h>
#include <iostream>
#include "engine/dx12_core.h"
#include "ui/ui_context.h"
#include <memory>

std::unique_ptr<DX12Core> g_dx12;
std::unique_ptr<UIContext> g_ui;

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
        CW_USEDEFAULT, CW_USEDEFAULT, 1280, 800,
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
        
        g_dx12 = std::make_unique<DX12Core>(hwnd, width, height);
        g_ui = std::make_unique<UIContext>(hwnd, g_dx12.get());
        
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
                // UI Frame
                g_ui->BeginFrame();
                g_ui->Render(); 
                g_ui->EndFrame();

                // Render DX12
                g_dx12->Render([&](ID3D12GraphicsCommandList* cmdList) {
                    // Render ImGui into command list
                    ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), cmdList);
                    
                    // Update and Render additional Platform Windows (Docking)
                    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
                    {
                        ImGui::UpdatePlatformWindows();
                        ImGui::RenderPlatformWindowsDefault();
                    }
                });
            }
        }
    }
    
    // Cleanup
    g_ui.reset();
    g_dx12.reset();

    return 0;
}
