#include <array>
#include <cmath>
#include <iomanip>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "renderer.hpp"

using namespace Rndr;

#define INF std::numeric_limits<double>::infinity()
Rndr::Renderer::~Renderer() {
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}
void Rndr::Renderer::draw() {
    while (!glfwWindowShouldClose(window_)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }
}
void Rndr::Renderer::initWindow(const std::string &title) {
    if (glfwInit() == 0) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    window_ =
        glfwCreateWindow(width_, height_, title.c_str(), nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(window_);
    if (vao_ || vbo_) {
        return;
    }
}
