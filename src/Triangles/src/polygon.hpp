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
    static constexpr size_t POINT_NUM = 2;

    union Vec_t {
        struct {
            Point_t v0_;
            Point_t v1_;
        };
        Point_t points[POINT_NUM];

        Vec_t(Point_t p1, Point_t p2) : v0_(p1), v1_(p2) {}

        Vec_t(const std::array<Point_t, POINT_NUM> &points)
            : v0_(points[0]), v1_(points[1]) {}
    } data_;

  public:
    Vector_t(const std::array<Point_t, POINT_NUM> &points) : data_(points) {}
    // TODO: {initializer}
    Vector_t(const Point_t point = {0, 0, 0})
        : data_(Point_t{0, 0, 0}, point) {}

    Vector_t operator+(const Point_t point) const noexcept {
        return Vector_t(Point_t{v1().x() + point.x(), v1().y() + point.y(),
                                v1().z() + point.z()});
    }

    Vector_t operator*(const double coef) const noexcept {
        return Vector_t(
            Point_t{v1().x() * coef, v1().y() * coef, v1().z() * coef});
    }

    Point_t &operator[](size_t i) { return data_.points[i]; }
    const Point_t &operator[](size_t i) const { return data_.points[i]; }

    static Vector_t CrossProduct(const Vector_t &vec1, const Vector_t &vec2) {
        double c1 =
            vec1.v1().y() * vec2.v1().z() - vec2.v1().y() * vec1.v1().z();
        double c2 =
            -(vec1.v1().x() * vec2.v1().z() - vec2.v1().x() * vec1.v1().z());
        double c3 =
            vec1.v1().x() * vec2.v1().y() - vec2.v1().x() * vec1.v1().y();
        return Vector_t(Point_t{c1, c2, c3});
    }

    static double DotProduct(const Vector_t &vec1, const Vector_t &vec2) {
        return vec1.v1().x() * vec2.v1().x() + vec1.v1().y() * vec2.v1().y() +
               vec1.v1().z() * vec2.v1().z();
    }

    const Point_t v0() const { return data_.v0_; }
    const Point_t v1() const { return data_.v1_; }

    // TODO: probably add x() y() z() that will return v1().x()
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

    // TODO: ERRORS DO NOT EXIST?
    double SignedDistance(const Vector_t &norm, const Point_t x,
                          const double shift) const {
        auto dist = Vector_t::DotProduct(norm, x) + shift;
        return (std::abs(dist) < EPS) ? 0.0 : dist;
    }

    std::array<double, POINT_NUM>
    ComputeDistances(const Polygon_t &poly) const {
        auto norm = Vector_t::CrossProduct(Vector_t(v2() - v1()),
                                           Vector_t(v2() - v0()));
        auto d2 = -Vector_t::DotProduct(norm, Vector_t{v0()});

        return std::array<double, POINT_NUM>{
            SignedDistance(norm, poly.v0(), d2),
            SignedDistance(norm, poly.v1(), d2),
            SignedDistance(norm, poly.v2(), d2)};
    }

    static double ComputeProjection(const Vector_t &axis, const Vector_t &vec) {
        auto max_component =
            std::max({axis.v1().x(), axis.v1().y(), axis.v1().z()});

        return (max_component == axis.v1().x())   ? vec.v1().x()
               : (max_component == axis.v1().y()) ? vec.v1().y()
                                                  : vec.v1().z();
    }

    std::pair<double, double>
    ComputeIntersectionInterval(const double dist_i, const double dist_j,
                                const double proj_i,
                                const double proj_j) const {
        double t_max = std::numeric_limits<double>::min();
        double t_min = std::numeric_limits<double>::max();

        auto s =
            std::max(dist_i / (dist_i - dist_j), dist_j / (dist_i - dist_j));
        t_max = std::max(proj_i + s * (proj_j - proj_i), t_max);
        t_min = std::min(proj_i + s * (proj_j - proj_i), t_min);

        return {t_min, t_max};
    }

    // FIXME: I do not really understand what happens here
    std::pair<double, double> ComputeIntersectionIntervals(
        const Vector_t &axis,
        const std::array<double, POINT_NUM> &distances) const {
        std::array<double, POINT_NUM> projections{
            ComputeProjection(axis, Vector_t{v0()}),
            ComputeProjection(axis, Vector_t{v1()}),
            ComputeProjection(axis, Vector_t{v2()})};

        std::pair<double, double> t;
        // determine which are on one side
        if (distances[0] * distances[1] >= 0) {
            t = ComputeIntersectionInterval(distances[0], distances[1],
                                            projections[0], projections[1]);
        } else if (distances[1] * distances[2] >= 0) {
            t = ComputeIntersectionInterval(distances[1], distances[2],
                                            projections[1], projections[2]);
        } else {
            t = ComputeIntersectionInterval(distances[2], distances[0],
                                            projections[2], projections[0]);
        }

        return t;
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

        if (std::max(this_interval.first, other_interval.first) <=
            std::min(this_interval.second, other_interval.second)) {
            std::cerr << "Complex\n";
        }
        return (std::max(this_interval.first, other_interval.first) <=
                std::min(this_interval.second, other_interval.second));
    }

    bool GeneralIntersectionCheck(const Polygon_t &poly) {
        std::array<double, POINT_NUM> this_distances = ComputeDistances(poly);
        std::array<double, POINT_NUM> other_distances =
            poly.ComputeDistances(*this);

        if (std::all_of(this_distances.begin(), this_distances.end(),
                        [](auto element) { return (element > EPS); })) {
            return false;
        } else if (std::all_of(this_distances.begin(), this_distances.end(),
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
                dirs[i] = Vector_t::DotProduct(sides[i], center);
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
