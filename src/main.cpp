#include <array>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <vector>

#include "Triangles/src/polygon.hpp"

int main() {
    size_t poly_num;
    std::cin >> poly_num;
    assert(poly_num > 0 && "amount of triangles must be non-negative");

    std::vector<Point_t> points;
    for (size_t i = 0; i < poly_num * 3; ++i) {
        double x, y, z;
        std::cin >> x >> y >> z;
        points.push_back({x, y, z});
    }

    std::vector<Polygon_t> polygons;
    for (size_t i = 0; i < points.size() - 2; i += 3) {
        std::array<Point_t, 3> pts = {points[i], points[i + 1], points[i + 2]};
        polygons.emplace_back(pts);
    }

    std::vector<bool> ints(poly_num, false);
    // #pragma omp parallel for
    for (size_t i = 0; i < polygons.size(); ++i) {
        if (ints[i]) {
            continue;
        }
        for (size_t j = i + 1; j < polygons.size(); ++j) {
            if (ints[j]) {
                continue;
            }
            if (polygons[i].GeneralIntersectionCheck(polygons[j])) {
                ints[i] = true;
                ints[j] = true;
            }
        }
    }

    for (size_t i = 0; i < poly_num; ++i) {
        if (ints[i]) {
            std::cout << i + 1 << " ";
        }
    }
    std::cout << std::endl;
}