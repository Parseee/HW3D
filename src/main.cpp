#include <array>
#include <cassert>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <unordered_set>
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

    system("python3 draw_debug.py");
}

int main() {
    int n;
    std::cin >> n;
    assert(n > 0 && "amount of triangles must be non-negative");

    std::vector<Point_t> points;
    for (size_t i = 0; i < size_t(n); ++i) {
        double x, y, z;
        std::cin >> x >> y >> z;
        points.push_back({x, y, z});
    }

    std::vector<Polygon_t> polygons;
    for (size_t i = 0; i < points.size() - 2; i += 3) {
        std::array<Point_t, 3> pts = {points[i], points[i + 1], points[i + 2]};
        polygons.emplace_back(pts);
    }

    std::unordered_set<size_t> ints;
    for (size_t i = 0; i < polygons.size(); ++i) {
        for (size_t j = i + 1; j < polygons.size(); ++j) {
            if (polygons[i].GeneralIntersectionCheck(polygons[j])) {
                ints.insert(i);
                ints.insert(j);
                // std::cout << polygons[i] << " " << polygons[j] << std::endl;
                // break;
            }
        }
    }

    for (const auto &elem : ints) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    exportOBJ(points, n / 9, "triangles.obj");
}