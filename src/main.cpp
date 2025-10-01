#include <array>
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

    std::cerr << polygons.size() << std::endl;

    size_t ints = 0;
    for (size_t i = 0; i < polygons.size(); ++i) {
        for (size_t j = i + 1; j < polygons.size(); ++j) {
            if (polygons[i].GeneralIntersectionCheck(polygons[j])) {
                ints += 1;
                // std::cout << polygons[i] << " " << polygons[j] << std::endl;
                break;
            }
        }
    }
    std::cout << ints << std::endl;

    // exportOBJ(points, n / 9, "triangles.obj");
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

/*
6
-21.1391
-2.96044
-3.74303
-21.487
-3.0482
-3.85087
-21.1587
-2.62302
-4.02156
16.086
2.91767
11.365
16.3106
2.91342
11.0859
16.7994
2.65756
10.6859
*/