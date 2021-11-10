//
// Created by Edge on 2020/12/23.
//

#include "glad/glad.h"
#include "ShaderProgram.h"
#include "Pipeline.h"
#include <Timer.h>
#ifdef __MINGW32__
#include <unistd.h>
#else
#include <thread>
#include <chrono>
#endif

#include <random>
#include <wx/wx.h>
#include "wx/graphics.h"

const char Pipeline::GLSL_VERSION[] = "#version 450";

void Pipeline::setupEnvironment() {
//    if (!glfwInit()) {
//        std::cerr << "[GLFW] init Failed" << std::endl;
//        exit(1);
//    }
//
//    if (!m_scene) {
//        exit(1);
//    }
//
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
//    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//
//    m_window = glfwCreateWindow(m_camera->m_width, m_camera->m_height, "Ray", NULL,
//                                NULL);
//
//    if (m_window == nullptr) {
//        std::cout << "[GLFW] failed to create window" << std::endl;
//        glfwTerminate();
//        exit(1);
//    }
//    glfwMakeContextCurrent(m_window);
//
//    // Deprecated
////    glewInit();
//    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
//        std::cout << "Failed to initialize GLAD" << std::endl;
//        exit(1);
//    }
//
//    printGraphicCardInfo();
//
//    setupGUIEnvironment();
}

void Pipeline::setupGUIEnvironment() {
//    IMGUI_CHECKVERSION();
//    ImGui::CreateContext();
//    ImGuiIO &io = ImGui::GetIO();
//    (void) io;
//    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
//    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
//
//    // Setup Dear ImGui style
//    ImGui::StyleColorsDark();
//    //ImGui::StyleColorsClassic();
//
//    // Setup Platform/Renderer backends
//    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
//    ImGui_ImplOpenGL3_Init(GLSL_VERSION);
}

void LocalRenderingPipeline::setupGUILayout() {
//    ImGui_ImplOpenGL3_NewFrame();
//    ImGui_ImplGlfw_NewFrame();
//    ImGui::NewFrame();
//
//    ImGui::Begin(
//            "Local rendering Pipeline Window");   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
//    ImGui::Text("Hello from another window!");
//    ImGui::End();
//
//    ImGui::Render();
//    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

}

void LocalRenderingPipeline::pipelineRender() {
//    glfwPollEvents();

    glEnable(GL_DEPTH_TEST);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderAllPass();

#ifdef GUI_SUPPORT
    setupGUILayout();
#endif

//    glfwSwapBuffers(m_window);
}

void LocalRenderingPipeline::pipelineLoop() {
//    while (!glfwWindowShouldClose(m_window)) {
//        glfwPollEvents();
//        glEnable(GL_DEPTH_TEST);
//        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//        renderAllPass();
//
//#ifdef GUI_SUPPORT
//        setupGUILayout();
//#endif
//
//        glfwSwapBuffers(m_window);
//    }
//
//    glfwTerminate();

}

