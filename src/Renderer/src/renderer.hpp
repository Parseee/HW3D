#pragma once

#include <algorithm>
#include <array>

#include <compare>
#include <iostream>
#include <ostream>
#include <string>

// https://connect.stealthsurf.app/to/691a0e1c8b62c996455777b9

#include <GLFW/glfw3.h>
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>

namespace Rndr {
class Renderer final {

  public:
    Renderer(size_t width, size_t height, const std::string &title)
        : width_(width), height_(height), window_(nullptr), vao_(0), vbo_(0) {
        initWindow(title);
        initGL();
    }

    ~Renderer() { glfwTerminate(); }

    Renderer(const std::vector<GLfloat> &arr) : points(arr) {}

    GLfloat &operator[](size_t i) { return points[i]; }

    const std::vector<GLfloat> &GetPoints() const { return points; }

    void draw() {
        glfwMakeContextCurrent(window_);
        while (!glfwWindowShouldClose(window_)) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glfwSwapBuffers(window_);
            glfwPollEvents();
        }
    }

  private:
    void initWindow(const std::string &title) {
        if (!glfwInit()) {
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

    void initGL() {}

    std::vector<GLfloat> points;
    size_t width_;
    size_t height_;
    GLFWwindow *window_;
    GLuint vao_, vbo_;
    // GLuint shader_;
};

}; // namespace Rndr
