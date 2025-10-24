#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>

#include <gtest/gtest.h>
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

    Point_t operator+(const Point_t &point) const {
        return Point_t(x() + point.x(), y() + point.y(), z() + point.z());
    }

    Point_t operator-(const Point_t &point) const {
        return Point_t(x() - point.x(), y() - point.y(), z() - point.z());
    }

    Point_t operator-() const { return Point_t(-x(), -y(), -z()); }

    double x() const { return data_.x_; }
    double y() const { return data_.y_; }
    double z() const { return data_.z_; }

    double &operator[](size_t i) { return data_.points[i]; }
    const double &operator[](size_t i) const { return data_.points[i]; }

    const double *data() const { return data_.points; }
};

class Vector_t {
  private:
    static constexpr size_t POINT_NUM = 1;

    Point_t v_;

  public:
    // TODO: {initializer}
    Vector_t(const Point_t point = {0, 0, 0}) : v_(point) {}

    Vector_t operator+(const Point_t point) const noexcept {
        return Vector_t(Point_t{x() + point.x(), y() + point.y(),
                                z() + point.z()});
    }

    Vector_t operator*(const double coef) const noexcept {
        return Vector_t(
            Point_t{x() * coef, y() * coef, z() * coef});
    }

    static Vector_t CrossProduct(const Vector_t &vec1, const Vector_t &vec2) {
        double c1 =
            vec1.y() * vec2.z() - vec2.y() * vec1.z();
        double c2 =
            -(vec1.x() * vec2.z() - vec2.x() * vec1.z());
        double c3 =
            vec1.x() * vec2.y() - vec2.x() * vec1.y();
        return Vector_t(Point_t{c1, c2, c3});
    }

    static double DotProduct(const Vector_t &vec1, const Vector_t &vec2) {
        return vec1.x() * vec2.x() + vec1.y() * vec2.y() +
               vec1.z() * vec2.z();
    }

    double x() const { return v_.x(); }
    double y() const { return v_.y(); }
    double z() const { return v_.z(); }
};

// FIXME: remove the fuckery with mixed {} and () initializations
class Polygon_t {
  private:
    static constexpr size_t POINT_NUM = 3;

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

    friend std::ostream &operator<<(std::ostream &stream,
                                    const Polygon_t &poly) {
        stream << '(';
        for (size_t i = 0; i < POINT_NUM; ++i) {
            stream << '(' << poly[i].x() << "," << poly[i].y() << ","
                   << poly[i].z() << ')';
            if (i != POINT_NUM - 1) {
                stream << ',';
            }
        }
        stream << ')';
        return stream;
    }

    // TODO: noexcept?
    double SignedDistance(const Vector_t &norm, const Point_t x,
                          const double shift) const {
        auto dist = Vector_t::DotProduct(norm, x) + shift;
        return (std::abs(dist) < EPS) ? 0.0 : dist;
    }

    std::array<double, POINT_NUM>
    ComputeDistances(const Polygon_t &poly) const {
        auto norm = Vector_t::CrossProduct(Vector_t(v1() - v0()),
                                           Vector_t(v2() - v0()));
        auto d2 = -Vector_t::DotProduct(norm, Vector_t{v0()});

        return std::array<double, POINT_NUM>{
            SignedDistance(norm, poly.v0(), d2),
            SignedDistance(norm, poly.v1(), d2),
            SignedDistance(norm, poly.v2(), d2)};
    }

    static double ComputeProjection(const Vector_t &axis, const Vector_t &vec) {
        double ax = axis.x(), ay = axis.y(), az = axis.z();
        double adx = std::abs(ax), ady = std::abs(ay), adz = std::abs(az);

        if (adx >= ady && adx >= adz)
            return vec.x();
        if (ady >= adx && ady >= adz)
            return vec.y();
        return vec.z();
    }

    std::pair<double, double> ComputeIntersectionIntervals(
        const Vector_t &axis, const std::array<double, POINT_NUM> &dist) const {
        std::array<double, POINT_NUM> proj{
            ComputeProjection(axis, Vector_t{v0()}),
            ComputeProjection(axis, Vector_t{v1()}),
            ComputeProjection(axis, Vector_t{v2()})};

        std::pair<double, double> t;
        if (dist[0] * dist[1] > 0) {
            t.first =
                proj[2] + (proj[0] - proj[2]) * (dist[2] / (dist[2] - dist[0]));
            t.second =
                proj[2] + (proj[1] - proj[2]) * (dist[2] / (dist[2] - dist[1]));
        } else if (dist[1] * dist[2] >= 0) {
            t.first =
                proj[0] + (proj[1] - proj[0]) * (dist[0] / (dist[0] - dist[1]));
            t.second =
                proj[0] + (proj[2] - proj[0]) * (dist[0] / (dist[0] - dist[2]));
        } else {
            t.first =
                proj[1] + (proj[2] - proj[1]) * (dist[1] / (dist[1] - dist[2]));
            t.second =
                proj[1] + (proj[0] - proj[1]) * (dist[1] / (dist[1] - dist[0]));
        }

        return {std::min(t.first, t.second), std::max(t.first, t.second)};
    }

    bool ComplexIntersectionCheck(
        const Polygon_t &poly,
        const std::array<double, POINT_NUM> &this_distances,
        const std::array<double, POINT_NUM> &other_distances) const {
        auto norm1 = Vector_t::CrossProduct(Vector_t(v2() - v1()),
                                            Vector_t(v2() - v0()));
        auto norm2 = Vector_t::CrossProduct(Vector_t(poly.v2() - poly.v1()),
                                            Vector_t(poly.v2() - poly.v0()));

        auto intersect_norm = Vector_t::CrossProduct(norm1, norm2);

        auto this_interval =
            ComputeIntersectionIntervals(intersect_norm, this_distances);
        auto other_interval =
            poly.ComputeIntersectionIntervals(intersect_norm, other_distances);

        auto left_border = std::max(this_interval.first, other_interval.first);
        auto right_border =
            std::min(this_interval.second, other_interval.second);
        return left_border <= right_border;
    }

    bool GeneralIntersectionCheck(const Polygon_t &poly) {
        std::array<double, POINT_NUM> this_distances = ComputeDistances(poly);
        std::array<double, POINT_NUM> other_distances =
            poly.ComputeDistances(*this);

        if (std::all_of(this_distances.begin(), this_distances.end(),
                        [](auto element) { return (element > EPS); }) ||
            std::all_of(other_distances.begin(), other_distances.end(),
                        [](auto element) { return (element > EPS); })) {
            return false;
        } else if (std::all_of(this_distances.begin(), this_distances.end(),
                               [](auto element) { return (-element > EPS); }) ||
                   std::all_of(other_distances.begin(), other_distances.end(),
                               [](auto element) { return (-element > EPS); })) {
            return false;
        } else if (std::all_of(
                       this_distances.begin(), this_distances.end(),
                       [](auto element) { return std::abs(element) < EPS; })) {
            return CoplanarIntersectionCheck(poly);
        }

        return ComplexIntersectionCheck(poly, this_distances, other_distances);
    }

    bool CheckPolygonInPolygon(const Polygon_t &poly) const {
        const std::array<Vector_t, POINT_NUM> sides{
            Vector_t({poly.v1() - poly.v0()}),
            Vector_t({poly.v2() - poly.v1()}),
            Vector_t({poly.v0() - poly.v2()})};

        for (size_t i = 0; i < POINT_NUM; ++i) {
            std::array<double, POINT_NUM> dirs;
            for (size_t j = 0; j < sides.size(); ++j) {
                Vector_t center = Vector_t(poly[i] - (*this)[j]);
                dirs[j] = Vector_t::DotProduct(sides[i], center);
            }
            if (!std::all_of(dirs.begin(), dirs.end(),
                             [](const auto &elem) { return elem > EPS; }) &&
                !std::all_of(dirs.begin(), dirs.end(),
                             [](const auto &elem) { return -elem > EPS; })) {
                return false;
            }
        }
        return true;
    }

    static bool CheckSegmentsIntersection(const Point_t &v1, const Point_t &v2,
                                          const Point_t &w1,
                                          const Point_t &w2) {

        auto int1 = Vector_t::DotProduct(Vector_t{v2 - v1}, Vector_t{w2 - v1}) *
                    Vector_t::DotProduct(Vector_t{v2 - v1}, Vector_t{w1 - v1});

        auto int2 = Vector_t::DotProduct(Vector_t{w2 - w1}, Vector_t{v2 - w1}) *
                    Vector_t::DotProduct(Vector_t{w2 - w1}, Vector_t{v1 - w1});

        return (-int1 > EPS) && (-int2 > EPS);
    }

    bool CheckPolygonSidesIntersection(const Polygon_t &poly) const {

        for (size_t i = 0; i < POINT_NUM; ++i) {
            for (size_t j = 0; j < POINT_NUM; ++j) {
                if (!CheckSegmentsIntersection(poly[i], poly[j], (*this)[i],
                                               (*this)[j])) {
                    return false;
                }
            }
        }
        return true;
    }

    bool CoplanarIntersectionCheck(const Polygon_t &poly) const {
        // TODO: add AABB for speed

        if (CheckPolygonInPolygon(poly) || poly.CheckPolygonInPolygon(*this)) {
            std::cerr << "Coplanar\n";
            return true;
        }

        if (CheckPolygonSidesIntersection(poly)) {
            std::cerr << "Coplanar\n";
            return true;
        }

        return false;
    }

  private:
};
