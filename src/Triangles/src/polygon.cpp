#include <iomanip>

#include "polygon.hpp"

double Geom::Polygon_t::SignedDistance(const Geom::Vector_t &norm,
                                       const Point_t x,
                                       const double shift) const noexcept {
    auto dist = Geom::Vector_t::DotProduct(norm, x) + shift;
    return (std::abs(dist) < EPS) ? 0.0 : dist;
}

std::array<double, Geom::Polygon_t::POINT_NUM>
Geom::Polygon_t::ComputeDistances(const Geom::Polygon_t &poly) const {
    auto norm = Geom::Vector_t::CrossProduct(Geom::Vector_t(v1() - v0()),
                                             Geom::Vector_t(v2() - v0()));
    auto d2 = -Geom::Vector_t::DotProduct(norm, Geom::Vector_t{v0()});

    return std::array<double, POINT_NUM>{SignedDistance(norm, poly.v0(), d2),
                                         SignedDistance(norm, poly.v1(), d2),
                                         SignedDistance(norm, poly.v2(), d2)};
}

double Geom::Polygon_t::ComputeProjection(const Geom::Vector_t &axis,
                                          const Geom::Vector_t &vec) {
    double ax = axis.x(), ay = axis.y(), az = axis.z();
    double adx = std::abs(ax), ady = std::abs(ay), adz = std::abs(az);

    if (adx >= ady && adx >= adz)
        return vec.x();
    if (ady >= adx && ady >= adz)
        return vec.y();
    return vec.z();
}

std::pair<double, double> Geom::Polygon_t::ComputeIntersectionIntervals(
    const Geom::Vector_t &axis,
    const std::array<double, POINT_NUM> &dist) const {
    std::array<double, POINT_NUM> proj{
        ComputeProjection(axis, Geom::Vector_t{v0()}),
        ComputeProjection(axis, Geom::Vector_t{v1()}),
        ComputeProjection(axis, Geom::Vector_t{v2()})};

    std::vector<double> intersections;

    for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;

        if (dist[i] * dist[j] < 0.0) {
            double t = dist[i] / (dist[i] - dist[j]);
            intersections.push_back(proj[i] + t * (proj[j] - proj[i]));
        } else if (std::abs(dist[i]) < EPS) {
            intersections.push_back(proj[i]);
        }
    }

    if (intersections.size() >= 2) {
        std::sort(intersections.begin(), intersections.end());
        return {intersections.front(), intersections.back()};
    }

    return {0.0, 0.0};
}

bool Geom::Polygon_t::ComplexIntersectionCheck(
    const Geom::Polygon_t &poly,
    const std::array<double, POINT_NUM> &this_distances,
    const std::array<double, POINT_NUM> &other_distances) const {
    auto norm1 = Geom::Vector_t::CrossProduct(Geom::Vector_t(v2() - v0()),
                                              Geom::Vector_t(v1() - v0()));
    auto norm2 =
        Geom::Vector_t::CrossProduct(Geom::Vector_t(poly.v2() - poly.v0()),
                                     Geom::Vector_t(poly.v1() - poly.v0()));

    auto intersect_norm = Geom::Vector_t::CrossProduct(norm1, norm2);

    auto this_interval =
        ComputeIntersectionIntervals(intersect_norm, other_distances);
    auto other_interval =
        poly.ComputeIntersectionIntervals(intersect_norm, this_distances);

    auto left_border = std::max(this_interval.first, other_interval.first);
    auto right_border = std::min(this_interval.second, other_interval.second);
    return right_border >= left_border;
}

bool Geom::Polygon_t::GeneralIntersectionCheck(const Geom::Polygon_t &poly) {
    // from poly to plane of this
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

bool Geom::Polygon_t::CheckPolygonInPolygon(const Geom::Polygon_t &poly) const {
    const Geom::Vector_t normal = Geom::Vector_t::CrossProduct(
        poly.v1() - poly.v0(), poly.v2() - poly.v0());
    bool inside = true;

    for (size_t i = 0; i < POINT_NUM; ++i) {
        const Point_t &point = (*this)[i];

        for (size_t j = 0; j < POINT_NUM; ++j) {
            const Point_t &a = poly[j];
            const Point_t &b = poly[(i + 1) % POINT_NUM];

            const Geom::Vector_t dir = b - a;
            const Geom::Vector_t edge_normal =
                Geom::Vector_t::CrossProduct(normal, dir);

            if (Geom::Vector_t::DotProduct(edge_normal, point - a) > EPS) {
                inside = false;
                break;
            }
        }
        if (!inside) {
            return false;
        }
    }
    return inside;
}

bool Geom::Polygon_t::CheckSegmentsIntersection(const Point_t &v1,
                                                const Point_t &v2,
                                                const Point_t &w1,
                                                const Point_t &w2) {

    auto int1 = Geom::Vector_t::DotProduct(Geom::Vector_t{v2 - v1},
                                           Geom::Vector_t{w2 - v1}) *
                Geom::Vector_t::DotProduct(Geom::Vector_t{v2 - v1},
                                           Geom::Vector_t{w1 - v1});

    auto int2 = Geom::Vector_t::DotProduct(Geom::Vector_t{w2 - w1},
                                           Geom::Vector_t{v2 - w1}) *
                Geom::Vector_t::DotProduct(Geom::Vector_t{w2 - w1},
                                           Geom::Vector_t{v1 - w1});

    return (-int1 > EPS) && (-int2 > EPS);
}

bool Geom::Polygon_t::CheckPolygonSidesIntersection(
    const Geom::Polygon_t &poly) const {
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

bool Geom::Polygon_t::CoplanarIntersectionCheck(
    const Geom::Polygon_t &poly) const {
    if (CheckPolygonInPolygon(poly) || poly.CheckPolygonInPolygon(*this)) {
        return true;
    }

    if (CheckPolygonSidesIntersection(poly)) {
        return true;
    }

    return false;
}

std::ostream &operator<<(std::ostream &stream, const Geom::Polygon_t &poly) {
    stream << "triangle(";
    stream << std::fixed << std::setprecision(2);

    for (size_t i = 0; i < poly.POINT_NUM; ++i) {
        const auto &p = poly[i];
        stream << "(" << p.x() << ", " << p.y() << ", " << p.z() << ")";
        if (i != poly.POINT_NUM - 1)
            stream << ',';
    }

    stream << ")";
    return stream;
}

Geom::Vector_t Geom::Vector_t::CrossProduct(const Vector_t &vec1,
                                            const Vector_t &vec2) {
    double c1 = vec1.y() * vec2.z() - vec2.y() * vec1.z();
    double c2 = -(vec1.x() * vec2.z() - vec2.x() * vec1.z());
    double c3 = vec1.x() * vec2.y() - vec2.x() * vec1.y();
    return Vector_t(Point_t{c1, c2, c3});
}

std::ostream &Geom::operator<<(std::ostream &out, const Vector_t &vec) {
    out << "vector((0,0,0), (" << std::fixed << std::setprecision(2) << vec.x()
        << ", " << vec.y() << ", " << vec.z() << "))";
    return out;
}

Geom::Vector_t Geom::operator*(const Vector_t &vec,
                               const double coef) noexcept {
    return Vector_t(Point_t{vec.x() * coef, vec.y() * coef, vec.z() * coef});
}

Geom::Vector_t Geom::operator+(const Vector_t &vec,
                               const Point_t &point) noexcept {
    return Vector_t(
        Point_t{vec.x() + point.x(), vec.y() + point.y(), vec.z() + point.z()});
}

Geom::AABB Geom::Polygon_t::GetAABB() const {
    AABB result;

    result.lower_bound = std::min({v0(), v1(), v2()});
    result.upper_bound = std::max({v0(), v1(), v2()});

    return result;
}
