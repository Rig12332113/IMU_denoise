#include <cmath>
#include <cstdio>
#include <array>

#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "implot.h"

int main() {
    // 1) GLFW init
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    // macOS OpenGL 3.2 core
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(900, 700, "IMU Viewer Test (ImPlot)", nullptr, nullptr);
    if (!window) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 2) ImGui init
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // 3) ImPlot init
    ImPlot::CreateContext();

    // 4) Dummy data buffers (last 150 samples)
    static constexpr int N = 150;
    std::array<float, N> x{};
    std::array<float, N> y{};

    for (int i = 0; i < N; ++i) {
        x[i] = static_cast<float>(i);
        y[i] = 0.0f;
    }

    float phase = 0.0f;

    // 5) Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Update dummy signal (simulate “streaming” by changing phase)
        phase += 0.05f;
        for (int i = 0; i < N; ++i) {
            float t = (static_cast<float>(i) * 0.1f) + phase;
            y[i] = std::sin(t);
        }

        // Start frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // UI
        ImGui::Begin("IMU Viewer");
        ImGui::Text("ImPlot test: 150-sample dummy signal");

        // Plot region
        if (ImPlot::BeginPlot("Signal", ImVec2(-1, 300))) {
            // Fix axes so it doesn't autoscale/jitter
            ImPlot::SetupAxes("time", "value", ImPlotAxisFlags_NoTickLabels, 0);
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, N - 1, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -1.2, 1.2, ImGuiCond_Always);

            ImPlot::PlotLine("raw", x.data(), y.data(), N);
            ImPlot::EndPlot();
        }

        ImGui::End();

        // Render
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // 6) Cleanup (reverse order)
    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
