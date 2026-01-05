#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

// macOS sockets
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include "receiver.hpp"

// ----------------------
// Ring buffer (last 150)
// ----------------------
struct Ring150 {
    static constexpr int N = 150;
    std::array<float, N> data{};
    int head = 0;
    bool full = false;

    void push(float v) {
        data[head] = v;
        head = (head + 1) % N;
        if (head == 0) full = true;
    }

    // Copy out in time order: oldest -> newest. Returns sample count (<=150).
    int snapshot(std::array<float, N>& out) const {
        const int count = full ? N : head;
        if (!full) {
            for (int i = 0; i < count; ++i) out[i] = data[i];
            return count;
        }
        // oldest is head when full
        for (int i = 0; i < N; ++i) out[i] = data[(head + i) % N];
        return N;
    }
};

struct ImuRawBuffers {
    Ring150 ax, ay, az;
    std::mutex m;
};

// ----------------------
// TCP receiver thread
// ----------------------
static void tcp_receiver_thread(ImuRawBuffers* buf, std::atomic<bool>* running) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::fprintf(stderr, "[viewer] socket() failed\n");
        return;
    }

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in servaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) {
        std::fprintf(stderr, "[viewer] bind() failed on port %d (is IMU_server running?)\n", PORT);
        close(sockfd);
        return;
    }

    if (listen(sockfd, 1) != 0) {
        std::fprintf(stderr, "[viewer] listen() failed\n");
        close(sockfd);
        return;
    }

    std::fprintf(stderr, "[viewer] listening on TCP port %d ...\n", PORT);

    // Accept with select timeout so we can stop gracefully.
    int connfd = -1;
    while (running->load()) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sockfd, &rfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 200 * 1000; // 200 ms

        int ret = select(sockfd + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            std::fprintf(stderr, "[viewer] select() failed during accept\n");
            break;
        }
        if (ret == 0) continue; // timeout

        sockaddr_in cli{};
        socklen_t len = sizeof(cli);
        connfd = accept(sockfd, (SA*)&cli, &len);
        if (connfd >= 0) break;
    }

    if (connfd < 0) {
        close(sockfd);
        std::fprintf(stderr, "[viewer] stopped before client connected\n");
        return;
    }

    std::fprintf(stderr, "[viewer] client connected\n");

    std::string accum;
    accum.reserve(4096);

    char buf_read[MAX];

    // Read loop with select timeout so we can stop gracefully.
    while (running->load()) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(connfd, &rfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 200 * 1000; // 200 ms

        int ret = select(connfd + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            std::fprintf(stderr, "[viewer] select() failed during recv\n");
            break;
        }
        if (ret == 0) continue; // timeout

        ssize_t n = read(connfd, buf_read, sizeof(buf_read));
        if (n <= 0) {
            std::fprintf(stderr, "[viewer] connection closed\n");
            break;
        }

        accum.append(buf_read, buf_read + n);

        // process complete lines
        size_t pos;
        while ((pos = accum.find('\n')) != std::string::npos) {
            std::string line = accum.substr(0, pos);
            accum.erase(0, pos + 1);

            IMUsample sample;
            if (!parse_one_quat_accg(line, sample)) continue;

            const auto a = sample.getAccG(); // accel only
            {
                std::lock_guard<std::mutex> lk(buf->m);
                buf->ax.push(static_cast<float>(a[0]));
                buf->ay.push(static_cast<float>(a[1]));
                buf->az.push(static_cast<float>(a[2]));
            }
        }
    }

    close(connfd);
    close(sockfd);
    std::fprintf(stderr, "[viewer] receiver thread exit\n");
}

int main() {
    // ----------------------
    // GLFW + OpenGL init
    // ----------------------
    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    const char* glsl_version = "#version 150"; // macOS OpenGL
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(900, 800, "IMU Viewer (Raw Accel)", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // ----------------------
    // ImGui + ImPlot init
    // ----------------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImPlot::CreateContext();

    // ----------------------
    // Data + receiver thread
    // ----------------------
    ImuRawBuffers raw;
    std::atomic<bool> running{true};
    std::thread rx(tcp_receiver_thread, &raw, &running);

    // x-axis (0..149)
    static constexpr int N = Ring150::N;
    std::array<float, N> x{};
    for (int i = 0; i < N; ++i) x[i] = static_cast<float>(i);

    // plotting snapshots
    std::array<float, N> ax_s{}, ay_s{}, az_s{};
    int count = 0;

    // Y-axis range for acc_g units (adjust if needed)
    float y_max = 4.0f;

    // ----------------------
    // Main UI loop
    // ----------------------
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Snapshot buffers (lock briefly)
        {
            std::lock_guard<std::mutex> lk(raw.m);
            count = raw.ax.snapshot(ax_s);
            raw.ay.snapshot(ay_s);
            raw.az.snapshot(az_s);
        }

        // Window 1 plot
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        const ImGuiViewport* vp = ImGui::GetMainViewport();

        ImVec2 pos  = vp->WorkPos;   // WorkPos avoids menu bar/dock area
        ImVec2 pivot = ImVec2(0.0f, 0.0f);

        ImGui::SetNextWindowPos(pos, ImGuiCond_Always, pivot);
        ImGui::SetNextWindowSize(ImVec2(600, 800), ImGuiCond_Always);

        ImGuiWindowFlags flags =
            ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoResize
            | ImGuiWindowFlags_NoCollapse;

        ImGui::Begin("IMU Raw Accel", nullptr, flags);
        ImGui::Text("Listening on TCP port %d (stop IMU_server if it uses the same port).", PORT);
        ImGui::Text("Samples available: %d / %d", count, N);
        ImGui::SliderFloat("Y_max", &y_max, 0.0f, 10.0f);

        // ax
        if (ImPlot::BeginPlot("ax (g)", ImVec2(600, 180))) {
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, N - 1, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -y_max, y_max, ImGuiCond_Always);
            if (count > 1) ImPlot::PlotLine("ax", x.data(), ax_s.data(), count);
            ImPlot::EndPlot();
        }

        // ay
        if (ImPlot::BeginPlot("ay (g)", ImVec2(600, 180))) {
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, N - 1, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -y_max, y_max, ImGuiCond_Always);
            if (count > 1) ImPlot::PlotLine("ay", x.data(), ay_s.data(), count);
            ImPlot::EndPlot();
        }

        // az
        if (ImPlot::BeginPlot("az (g)", ImVec2(600, 180))) {
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, N - 1, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -y_max, y_max, ImGuiCond_Always);
            if (count > 1) ImPlot::PlotLine("az", x.data(), az_s.data(), count);
            ImPlot::EndPlot();
        }

        ImGui::End();
        
        // Window 2 numerical data
        ImGui::SetNextWindowPos(ImVec2(600, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_Always);

        ImGui::Begin("raw numerical data", nullptr, flags);
        if (count > 0) {
            float ax_last = ax_s[count - 1];
            float ay_last = ay_s[count - 1];
            float az_last = az_s[count - 1];
            ImGui::Text("ax: %.4f g", ax_last);
            ImGui::Text("ay: %.4f g", ay_last);
            ImGui::Text("az: %.4f g", az_last);
        } 
        else {
            ImGui::Text("Waiting for data...");
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

    // ----------------------
    // Shutdown
    // ----------------------
    running.store(false);
    if (rx.joinable()) rx.join();

    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
