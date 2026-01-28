#include <array>
#include <cmath>
#include <iomanip>

#include "polygon.hpp"

using namespace Geom;

#define INF std::numeric_limits<double>::infinity()

double Polygon_t::SignedDistance(const Vector_t &norm, const Point_t x,
                                 const double shift) const noexcept {
    auto dist = Vector_t::DotProduct(norm, x) + shift;
    return (std::abs(dist) < EPS) ? 0.0 : dist;
}

std::array<double, 3> Polygon_t::ComputeDistances(const Polygon_t &poly) const {
    Vector_t normal = Vector_t::CrossProduct(v1() - v0(), v2() - v0());

    if (normal == Vector_t({0, 0, 0})) {
        return {INF, INF, INF};
    }

    double planeOffset = -Vector_t::DotProduct(normal, v0());

    return {SignedDistance(normal, poly.v0(), planeOffset),
            SignedDistance(normal, poly.v1(), planeOffset),
            SignedDistance(normal, poly.v2(), planeOffset)};
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

std::pair<double, double>
Geom::Polygon_t::ComputeLineIntervals(const std::array<double, 3> &coords,
                     const std::array<double, 3> &dist) const {
    std::vector<double> values;

    for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;

        if (dist[i] * dist[j] < 0.0) {
            double t = dist[i] / (dist[i] - dist[j]);
            values.push_back(coords[i] + t * (coords[j] - coords[i]));
        } else if (dist[i] == 0.0) {
            values.push_back(coords[i]);
        }
    }

    if (values.size() < 2)
        return {INF, -INF};

    std::sort(values.begin(), values.end());
    return {values.front(), values.back()};
}

bool Polygon_t::ComplexIntersectionCheck(
    const Polygon_t &other, const std::array<double, 3> &thisDistances,
    const std::array<double, 3> &otherDistances) const {
    Vector_t normal1 = GetNormal();
    Vector_t normal2 = other.GetNormal();

    Vector_t intersectionDir = Vector_t::CrossProduct(normal1, normal2);

    size_t axis = intersectionDir.DominantAxis();

    std::array<double, 3> thisCoords = {v0()[axis], v1()[axis], v2()[axis]};

    std::array<double, 3> otherCoords = {other.v0()[axis], other.v1()[axis],
                                         other.v2()[axis]};

    auto thisInterval = ComputeLineIntervals(thisCoords, thisDistances);

    auto otherInterval = ComputeLineIntervals(otherCoords, otherDistances);

    return !(thisInterval.second < otherInterval.first ||
             otherInterval.second < thisInterval.first);
}

bool Polygon_t::GeneralIntersectionCheck(const Polygon_t &poly) {
    auto distancesToThisPlane = ComputeDistances(poly);
    if (distancesToThisPlane[0] * distancesToThisPlane[1] > 0.0 &&
        distancesToThisPlane[0] * distancesToThisPlane[2] > 0.0) {
        return false;
    }
    auto distancesToOtherPlane = poly.ComputeDistances(*this);
    if (distancesToOtherPlane[0] * distancesToOtherPlane[1] > 0.0 &&
        distancesToOtherPlane[0] * distancesToOtherPlane[2] > 0.0) {
        return false;
    }

    Vector_t intersectionDir =
        Vector_t::CrossProduct(GetNormal(), poly.GetNormal());

    if (intersectionDir.Len() < EPS) {
        return CoplanarIntersectionCheck(poly);
    }

    return ComplexIntersectionCheck(poly, distancesToOtherPlane,
                                    distancesToThisPlane);
}

bool Polygon_t::CheckPolygonInPolygon(const Polygon_t &poly) const {
    const Vector_t normal =
        Vector_t::CrossProduct(poly.v1() - poly.v0(), poly.v2() - poly.v0());
    bool inside = true;

    for (size_t i = 0; i < POINT_NUM; ++i) {
        const Point_t &point = (*this)[i];

        for (size_t j = 0; j < POINT_NUM; ++j) {
            const Point_t &a = poly[j];
            const Point_t &b = poly[(i + 1) % POINT_NUM];

            const Vector_t dir = b - a;
            const Vector_t edge_normal = Vector_t::CrossProduct(normal, dir);

            if (Vector_t::DotProduct(edge_normal, point - a) > EPS) {
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
            if (CheckSegmentsIntersection((*this)[i],
                                          (*this)[(i + 1) % POINT_NUM], poly[j],
                                          poly[(j + 1) % POINT_NUM])) {
                return true;
            }
        }
    }
    return false;
}

bool Polygon_t::CoplanarIntersectionCheck(const Polygon_t &poly) const {
    if (CheckPolygonInPolygon(poly) || poly.CheckPolygonInPolygon(*this)) {
        return true;
    }

    if (CheckPolygonSidesIntersection(poly)) {
        return true;
    }

    return false;
}

std::ostream &operator<<(std::ostream &stream, const Polygon_t &poly) {
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

Vector_t Vector_t::CrossProduct(const Vector_t &vec1, const Vector_t &vec2) {
    double c1 = vec1.y() * vec2.z() - vec2.y() * vec1.z();
    double c2 = -(vec1.x() * vec2.z() - vec2.x() * vec1.z());
    double c3 = vec1.x() * vec2.y() - vec2.x() * vec1.y();
    return Vector_t(Point_t{c1, c2, c3});
}

std::ostream &operator<<(std::ostream &out, const Vector_t &vec) {
    out << "vector((0,0,0), (" << std::fixed << std::setprecision(2) << vec.x()
        << ", " << vec.y() << ", " << vec.z() << "))";
    return out;
}

Vector_t operator*(const Vector_t &vec, const double coef) noexcept {
    return Vector_t(Point_t{vec.x() * coef, vec.y() * coef, vec.z() * coef});
}

Vector_t operator+(const Vector_t &vec, const Point_t &point) noexcept {
    return Vector_t(
        Point_t{vec.x() + point.x(), vec.y() + point.y(), vec.z() + point.z()});
}

AABB Polygon_t::GetAABB() const {
    AABB result;

    result.lower_bound = std::min({v0(), v1(), v2()});
    result.upper_bound = std::max({v0(), v1(), v2()});

    return result;
}
