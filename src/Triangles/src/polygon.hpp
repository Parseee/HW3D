#pragma once

#include <array>
#include <cstddef>
#include <format>

#include <iostream>
#include <ostream>

namespace {
static constexpr double EPS = 1e-5;
} // namespace

class Point_t {
  private:
    static constexpr size_t POINT_NUM = 3;

    union IntPt_t {
        struct {
            double x_;
            double y_;
            double z_;
        };
        double points[POINT_NUM];

        IntPt_t(double p1 = 0.0, double p2 = 0.0, double p3 = 0.0)
            : x_(p1), y_(p2), z_(p3) {}

        IntPt_t(const std::array<double, POINT_NUM> &arr)
            : x_(arr[0]), y_(arr[1]), z_(arr[2]) {}
    } data_;

  public:
    Point_t(double p1 = 0.0, double p2 = 0.0, double p3 = 0.0)
        : data_(p1, p2, p3) {}

    Point_t(const std::array<double, POINT_NUM> &arr) : data_(arr) {}

    double x() const { return data_.x_; }
    double y() const { return data_.y_; }
    double z() const { return data_.z_; }

    double &operator[](size_t i) { return data_.points[i]; }
    const double &operator[](size_t i) const { return data_.points[i]; }

    const double *data() const { return data_.points; }
};

inline Point_t operator+(const Point_t &p1, const Point_t &p2) {
    return Point_t(p1.x() + p2.x(), p1.y() + p2.y(), p1.z() + p2.z());
}

inline Point_t operator-(const Point_t &p1, const Point_t &p2) {
    return Point_t(p1.x() - p2.x(), p1.y() - p2.y(), p1.z() - p2.z());
}

inline Point_t operator-(const Point_t &point) {
    return Point_t(-point.x(), -point.y(), -point.z());
}

class Vector_t {
  private:
    static constexpr size_t POINT_NUM = 1;

    Point_t v_;

  public:
    Vector_t(const Point_t point = {0, 0, 0}) : v_(point) {}

    static Vector_t CrossProduct(const Vector_t &vec1, const Vector_t &vec2) {
        double c1 = vec1.y() * vec2.z() - vec2.y() * vec1.z();
        double c2 = -(vec1.x() * vec2.z() - vec2.x() * vec1.z());
        double c3 = vec1.x() * vec2.y() - vec2.x() * vec1.y();
        return Vector_t(Point_t{c1, c2, c3});
    }

    static double DotProduct(const Vector_t &vec1, const Vector_t &vec2) {
        return vec1.x() * vec2.x() + vec1.y() * vec2.y() + vec1.z() * vec2.z();
    }

    double x() const { return v_.x(); }
    double y() const { return v_.y(); }
    double z() const { return v_.z(); }
};

inline Vector_t operator+(const Vector_t &vec, const Point_t &point) noexcept {
    return Vector_t(
        Point_t{vec.x() + point.x(), vec.y() + point.y(), vec.z() + point.z()});
}

inline Vector_t operator*(const Vector_t &vec, const double coef) noexcept {
    return Vector_t(Point_t{vec.x() * coef, vec.y() * coef, vec.z() * coef});
}

inline std::ostream &operator<<(std::ostream &out, const Vector_t &vec) {
    out << std::format("vector((0,0,0), ({:.2f}, {:.2f}, {:.2f}))", vec.x(),
                       vec.y(), vec.z());
    return out;
}

class Polygon_t {
  public:
    static constexpr size_t POINT_NUM = 3;

  private:
    union Pol_t {
        struct {
            Point_t v0_;
            Point_t v1_;
            Point_t v2_;
        };
        Point_t points[POINT_NUM];

        Pol_t(Point_t p1, Point_t p2, Point_t p3) : v0_(p1), v1_(p2), v2_(p3) {}

        Pol_t(const std::array<Point_t, POINT_NUM> &points)
            : v0_(points[0]), v1_(points[1]), v2_(points[2]) {}
    } data_;

  public:
    Polygon_t(const std::array<Point_t, POINT_NUM> &points) : data_(points) {}

    const Point_t v0() const { return data_.v0_; }
    const Point_t v1() const { return data_.v1_; }
    const Point_t v2() const { return data_.v2_; }

    Point_t &operator[](size_t i) { return data_.points[i]; }
    const Point_t &operator[](size_t i) const { return data_.points[i]; }

    double SignedDistance(const Vector_t &norm, const Point_t x,
                          const double shift) const noexcept;

    std::array<double, POINT_NUM> ComputeDistances(const Polygon_t &poly) const;

    static double ComputeProjection(const Vector_t &axis, const Vector_t &vec);

    std::pair<double, double> ComputeIntersectionIntervals(
        const Vector_t &axis, const std::array<double, POINT_NUM> &dist) const;

    bool ComplexIntersectionCheck(
        const Polygon_t &poly,
        const std::array<double, POINT_NUM> &this_distances,
        const std::array<double, POINT_NUM> &other_distances) const;

    bool GeneralIntersectionCheck(const Polygon_t &poly);

    bool CheckPolygonInPolygon(const Polygon_t &poly) const;

    static bool CheckSegmentsIntersection(const Point_t &v1, const Point_t &v2,
                                          const Point_t &w1, const Point_t &w2);

    bool CheckPolygonSidesIntersection(const Polygon_t &poly) const;

    bool CoplanarIntersectionCheck(const Polygon_t &poly) const;
};
std::ostream &operator<<(std::ostream &stream, const Polygon_t &poly);
