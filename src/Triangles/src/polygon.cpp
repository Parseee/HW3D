#include "polygon.hpp"

double Polygon_t::SignedDistance(const Vector_t &norm, const Point_t x,
                                 const double shift) const noexcept {
    auto dist = Vector_t::DotProduct(norm, x) + shift;
    return (std::abs(dist) < EPS) ? 0.0 : dist;
}

std::array<double, Polygon_t::POINT_NUM>
Polygon_t::ComputeDistances(const Polygon_t &poly) const {
    auto norm =
        Vector_t::CrossProduct(Vector_t(v1() - v0()), Vector_t(v2() - v0()));
    auto d2 = -Vector_t::DotProduct(norm, Vector_t{v0()});

    return std::array<double, POINT_NUM>{SignedDistance(norm, poly.v0(), d2),
                                         SignedDistance(norm, poly.v1(), d2),
                                         SignedDistance(norm, poly.v2(), d2)};
}

double Polygon_t::ComputeProjection(const Vector_t &axis, const Vector_t &vec) {
    double ax = axis.x(), ay = axis.y(), az = axis.z();
    double adx = std::abs(ax), ady = std::abs(ay), adz = std::abs(az);

    if (adx >= ady && adx >= adz)
        return vec.x();
    if (ady >= adx && ady >= adz)
        return vec.y();
    return vec.z();
}

std::pair<double, double> Polygon_t::ComputeIntersectionIntervals(
    const Vector_t &axis, const std::array<double, POINT_NUM> &dist) const {
    std::array<double, POINT_NUM> proj{ComputeProjection(axis, Vector_t{v0()}),
                                       ComputeProjection(axis, Vector_t{v1()}),
                                       ComputeProjection(axis, Vector_t{v2()})};

    int crossing_count = 0;
    double intersections[2];

    for (int i = 0; i < 3; i++) {
        int j = (i + 1) % 3;

        if (dist[i] * dist[j] < 0) {
            double param = dist[i] / (dist[i] - dist[j]);
            intersections[crossing_count] =
                proj[i] + param * (proj[j] - proj[i]);
            crossing_count++;
        } else if (std::abs(dist[i]) < EPS) {
            intersections[crossing_count] = proj[i];
            crossing_count++;
        }
    }

    if (crossing_count == 2) {
        return {std::min(intersections[0], intersections[1]),
                std::max(intersections[0], intersections[1])};
    } else {
        return {0.0, 0.0};
    }
}

bool Polygon_t::ComplexIntersectionCheck(
    const Polygon_t &poly, const std::array<double, POINT_NUM> &this_distances,
    const std::array<double, POINT_NUM> &other_distances) const {
    auto norm1 =
        Vector_t::CrossProduct(Vector_t(v2() - v0()), Vector_t(v1() - v0()));
    auto norm2 = Vector_t::CrossProduct(Vector_t(poly.v2() - poly.v0()),
                                        Vector_t(poly.v1() - poly.v0()));

    auto intersect_norm = Vector_t::CrossProduct(norm1, norm2);

    auto this_interval =
        ComputeIntersectionIntervals(intersect_norm, other_distances);
    auto other_interval =
        poly.ComputeIntersectionIntervals(intersect_norm, this_distances);

    auto left_border = std::max(this_interval.first, other_interval.first);
    auto right_border = std::min(this_interval.second, other_interval.second);
    return right_border >= left_border;
}

bool Polygon_t::GeneralIntersectionCheck(const Polygon_t &poly) {
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

bool Polygon_t::CheckPolygonInPolygon(const Polygon_t &poly) const {
    const std::array<Vector_t, POINT_NUM> sides{
        Vector_t({poly.v1() - poly.v0()}), Vector_t({poly.v2() - poly.v1()}),
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

bool Polygon_t::CheckSegmentsIntersection(const Point_t &v1, const Point_t &v2,
                                          const Point_t &w1,
                                          const Point_t &w2) {

    auto int1 = Vector_t::DotProduct(Vector_t{v2 - v1}, Vector_t{w2 - v1}) *
                Vector_t::DotProduct(Vector_t{v2 - v1}, Vector_t{w1 - v1});

    auto int2 = Vector_t::DotProduct(Vector_t{w2 - w1}, Vector_t{v2 - w1}) *
                Vector_t::DotProduct(Vector_t{w2 - w1}, Vector_t{v1 - w1});

    return (-int1 > EPS) && (-int2 > EPS);
}

bool Polygon_t::CheckPolygonSidesIntersection(const Polygon_t &poly) const {
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

bool Polygon_t::CoplanarIntersectionCheck(const Polygon_t &poly) const {
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

std::ostream &operator<<(std::ostream &stream, const Polygon_t &poly) {
    stream << "triangle(";
    for (size_t i = 0; i < poly.POINT_NUM; ++i) {
        stream << std::format("({:.2f}, {:.2f}, {:.2f})", poly[i].x(),
                              poly[i].y(), poly[i].z());
        if (i != poly.POINT_NUM - 1) {
            stream << ',';
        }
    }
    stream << ")";
    return stream;
}
