#pragma once

#include <algorithm>
#include <array>

#include <compare>
#include <iostream>
#include <ostream>

namespace {
static constexpr double EPS = 1e-5;
} // namespace

namespace Geom {
class Point_t final {
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

    auto operator<=>(const Point_t &other) const {
        // for (size_t i = 0; i < POINT_NUM; ++i) {
        //     if ((*this)[i] < other[i]) {
        //         return std::strong_ordering::less;
        //     }
        //     if ((*this)[i] > other[i]) {
        //         return std::strong_ordering::greater;
        //     }
        // }
        // return std::strong_ordering::equal;
        return std::lexicographical_compare_three_way(
            data_.points, data_.points + POINT_NUM, other.data_.points,
            other.data_.points + POINT_NUM);
    }
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

class Vector_t final {
  private:
    static constexpr size_t POINT_NUM = 1;

    Point_t v_;

  public:
    Vector_t(const Point_t point = {0, 0, 0}) : v_(point) {}

    static Vector_t CrossProduct(const Vector_t &vec1, const Vector_t &vec2);

    static double DotProduct(const Vector_t &vec1, const Vector_t &vec2) {
        return vec1.x() * vec2.x() + vec1.y() * vec2.y() + vec1.z() * vec2.z();
    }

    double Len() const { return std::sqrt(x() * x() + y() * y() + z() * z()); }

    double x() const { return v_.x(); }
    double y() const { return v_.y(); }
    double z() const { return v_.z(); }

    auto operator<=>(const Vector_t &) const = default;
};

Vector_t operator+(const Vector_t &vec, const Point_t &point) noexcept;
Vector_t operator*(const Vector_t &vec, const double coef) noexcept;
std::ostream &operator<<(std::ostream &out, const Vector_t &vec);

struct AABB {
    Vector_t lower_bound;
    Vector_t upper_bound;

    static AABB Union(const AABB &left, const AABB &right) {
        AABB res;

        res.lower_bound = std::min(left.lower_bound, right.lower_bound);
        res.upper_bound = std::max(left.upper_bound, right.upper_bound);

        return res;
    }

    double Volume() const {
        return Vector_t::CrossProduct(lower_bound, upper_bound).Len() / 2.;
    }
};

class Polygon_t final {
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

    AABB GetAABB() const;
};

std::ostream &operator<<(std::ostream &stream, const Polygon_t &poly);

}; // namespace Geom

namespace BB {

struct Node {
    Geom::AABB bbox;
    // size_t obj_idx;
    // size_t par_idx;
    // size_t lhs_idx;
    // size_t rhs_idx;
    // bool is_leaf;
};

class Tree {
  public:
    Tree(const std::vector<Geom::Polygon_t> &polygons) {
        nodes_.resize(polygons.size());

        for (size_t i = 0; i < nodes_.size(); ++i) {
            nodes_[i].bbox = polygons[i].GetAABB();
        }
    }

    bool IntersectAABB(size_t left, size_t right) const {
        const Geom::AABB &l = nodes_[left].bbox;
        const Geom::AABB &r = nodes_[right].bbox;

        bool x_sec = (l.lower_bound.x() <= r.upper_bound.x()) &&
                     (l.upper_bound.x() >= r.lower_bound.x());
        bool y_sec = (l.lower_bound.y() <= r.upper_bound.y()) &&
                     (l.upper_bound.y() >= r.lower_bound.y());
        bool z_sec = (l.lower_bound.z() <= r.upper_bound.z()) &&
                     (l.upper_bound.z() >= r.lower_bound.z());

        return x_sec && y_sec && z_sec;
    }

  private:
    std::vector<Node> nodes_;
    size_t root_idx;
};
}; // namespace BB
