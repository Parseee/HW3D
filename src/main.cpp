#include <cassert>
#include <fstream>
#include <iostream>
#include <vector>

#include "Triangles/src/polygon.hpp"

void exportOBJ(const std::vector<Point_t> &points, int numTriangles,
               const std::string &filename) {
    std::ofstream file(filename);
    if (!file.is_open())
        return;

    // Write vertices
    for (const auto &p : points)
        file << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";

    // Write triangles (faces)
    for (int i = 0; i < numTriangles; ++i) {
        int idx = i * 3;
        file << "f " << idx + 1 << " " << idx + 2 << " " << idx + 3
             << "\n"; // OBJ is 1-indexed
    }

    file.close();

    system("python3 ../draw_debug.py");
}

int main() {
    int n = 6;
    // std::cin >> n;
    assert(n > 0 && "amount of triangles must be non-negative");

    std::vector<Point_t> points = {{0, 1, 0}, {0, 0, 0},  {1, 0, 0.5},
                                   {1, 1, 0}, {0, -1, 0}, {1, 1, 1}};
    // for (size_t i = 0; i < size_t(n); ++i) {
    //     double x, y, z;
    //     std::cin >> x >> y >> z;
    //     points.push_back({x, y, z});
    // }

    Polygon_t a({points[0], points[1], points[2]});
    Polygon_t b({points[0], points[1], points[2]});
    std::cout << a.GeneralIntersectionCheck(b);

    exportOBJ(points, 2, "triangles.obj");
}

/*
6
0 1 0
0 0 0
1 0 0.5
1 1 0
0 -1 0
1 1 1
*/