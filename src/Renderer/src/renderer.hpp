#pragma once

#include <string>
#include <vector>

using GLfloat = float;
using GLuint = uint;
struct GLFWwindow;

namespace Rndr {
class Renderer final {

  public:
    Renderer(size_t width, size_t height, const std::string &title)
        : width_(width), height_(height), window_(nullptr), vao_(0), vbo_(0) {
        initWindow(title);
        initGL();
    }

    ~Renderer();

    GLfloat &operator[](size_t i) { return points[i]; }

    const std::vector<GLfloat> &GetPoints() const { return points; }

    void GetPoints(const std::vector<GLfloat> &points) {}

    void draw();

  private:
    void initWindow(const std::string &title);

    void initGL() {}

    std::vector<GLfloat> points;
    size_t width_;
    size_t height_;
    GLFWwindow *window_;
    GLuint vao_, vbo_;
    // GLuint shader_;
};

}; // namespace Rndr
