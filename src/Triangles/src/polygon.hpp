#pragma once

#include <algorithm>
#include <array>
#include <limits>

#include <gtest/gtest.h>

namespace {
static constexpr double EPS = 1e-5;
} // namespace

class Point_t {
  private:
    static constexpr size_t POINT_NUM = 3;

  public:
    Point_t(double p1 = 0.0, double p2 = 0.0, double p3 = 0.0)
        : x_(p1), y_(p2), z_(p3) {}

    Point_t(const std::array<double, POINT_NUM> &point)
        : x_(point[0]), y_(point[1]), z_(point[2]) {}

    Point_t operator+(const Point_t &point) const {
        return Point_t(point.x_ + x_, point.y_ + y_, point.z_ + z_);
    }
    Point_t operator+(Point_t &point) {
        return Point_t(point.x_ + x_, point.y_ + y_, point.z_ + z_);
    }

    Point_t operator-() const { return Point_t(-x_, -y_, -z_); }

    Point_t operator-(const Point_t &point) const { return point + (-(*this)); }

    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }

  private:
    double x_;
    double y_;
    double z_;
};

class Vector_t : public Point_t {
  private:
    static constexpr size_t POINT_NUM = 2;

  public:
    Vector_t(const std::array<Point_t, POINT_NUM> &points)
        : v0_(points[0]), v1_(points[1]) {}
    // TODO: {initializer}
    Vector_t(const Point_t point = {0, 0, 0})
        : v0_(Point_t{0, 0, 0}), v1_(point) {}

    Vector_t operator+(const Point_t point) const noexcept {
        return Vector_t(Point_t{v1_.x() + point.x(), v1_.y() + point.y(),
                                v1_.z() + point.z()});
    }

    Vector_t operator*(const double coef) const noexcept {
        return Vector_t(
            Point_t{v1_.x() * coef, v1_.y() * coef, v1_.z() * coef});
    }

    static Vector_t CrossProduct(const Vector_t &vec1, const Vector_t &vec2) {
        double c1 = vec1.v1_.y() * vec2.v1_.z() - vec2.v1_.y() * vec1.v1_.z();
        double c2 =
            -(vec1.v1_.x() * vec2.v1_.z() - vec2.v1_.x() * vec1.v0_.z());
        double c3 = vec1.v1_.x() * vec2.v1_.y() - vec2.v1_.x() * vec1.v1_.y();
        return Vector_t(Point_t{c1, c2, c3});
    }

    static double DotProduct(const Vector_t &vec1, const Vector_t &vec2) {
        return vec1.v1_.x() * vec2.v1_.x() + vec1.v1_.y() * vec2.v1_.y() +
               vec1.v1_.z() * vec2.v1_.z();
    }

    const Point_t v0() const { return v0_; }
    const Point_t v1() const { return v1_; }

  private:
    Point_t v0_;
    Point_t v1_;
};

class Line_t : Point_t {
  private:
    static constexpr size_t POINT_NUM = 2;

  public:
    Line_t(const std::array<Point_t, POINT_NUM> &points)
        : v0(points[0]), v1(points[1]) {}

    Line_t(const Line_t &vec1, const Point_t) {}

  private:
    Point_t v0;
    Point_t v1;
};

// TODO: remove the fuckery with mixed {} and () initializations
class Polygon_t {
  private:
    static constexpr size_t VERT_NUM = 3;

    enum class PointLocation { ON_PLANE, BELOW_PLANE, ABOVE_PLANE };

  public:
    Polygon_t(const std::array<Point_t, VERT_NUM> &points)
        : v0(points[0]), v1(points[1]), v2(points[2]) {}

    // TODO: ERRORS DO NOT EXIST?
    double SignedDistance(const Vector_t &norm, const Point_t x,
                          const double shift) const {
        auto dist = Vector_t::DotProduct(norm, x) + shift;
        return (std::abs(dist) < EPS) ? 0.0 : dist;
    }

    std::array<double, VERT_NUM> ComputeDistances() const {
        auto norm =
            Vector_t::CrossProduct(Vector_t(v2 - v1), Vector_t(v2 - v0));
        auto d2 = -Vector_t::DotProduct(norm, Vector_t{v0});

        return std::array<double, VERT_NUM>{SignedDistance(norm, v0, d2),
                                            SignedDistance(norm, v1, d2),
                                            SignedDistance(norm, v2, d2)};
    }

    double ComputeProjection(const Vector_t &axis, const Vector_t &vec) const {
        auto max_component =
            std::max({axis.v1().x(), axis.v1().y(), axis.v1().z()});

        return (max_component == axis.v1().x())   ? vec.v1().x()
               : (max_component == axis.v1().y()) ? vec.v1().y()
                                                  : vec.v1().y();
    }

    bool CoplanarIntersectionCheck(const Polygon_t &poly) const {
        return false;
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
        const std::array<double, VERT_NUM> &distances) const {
        std::array<double, VERT_NUM> projections{
            ComputeProjection(axis, Vector_t{v0}),
            ComputeProjection(axis, Vector_t{v1}),
            ComputeProjection(axis, Vector_t{v2})};

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
        const std::array<double, VERT_NUM> &this_distances,
        const std::array<double, VERT_NUM> &other_distances) const {
        auto norm1 =
            Vector_t::CrossProduct(Vector_t(v2 - v1), Vector_t(v2 - v0));
        auto norm2 = Vector_t::CrossProduct(Vector_t(poly.v2 - poly.v1),
                                            Vector_t(poly.v2 - poly.v0));

        auto intersect_norm = Vector_t::CrossProduct(norm1, norm2);

        auto this_interval =
            ComputeIntersectionIntervals(intersect_norm, this_distances);
        auto other_interval =
            poly.ComputeIntersectionIntervals(intersect_norm, other_distances);

        return (std::min(this_interval.first, other_interval.first) <=
                std::max(this_interval.second, other_interval.second));
    }

    bool GeneralIntersectionCheck(const Polygon_t &poly) {
        std::array<double, VERT_NUM> this_distances = ComputeDistances();
        std::array<double, VERT_NUM> other_distances = poly.ComputeDistances();

        if (std::all_of(this_distances.begin(), this_distances.end(),
                        [](auto element) {
                            return ((element > EPS) || (-element > EPS));
                        })) {
            return false;
        } else if (std::all_of(
                       this_distances.begin(), this_distances.end(),
                       [](auto element) { return std::abs(element) < EPS; })) {
            return CoplanarIntersectionCheck(poly);
        }

        return ComplexIntersectionCheck(poly, this_distances, other_distances);
    }

  private:
    Point_t v0;
    Point_t v1;
    Point_t v2;
};
