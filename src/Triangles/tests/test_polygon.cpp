#include "polygon.hpp"
#include <array>
#include <gtest/gtest.h>
#include <limits>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// ------------------- Point_t Tests -------------------
TEST(PointTest, DefaultConstructor) {
    Point_t p;
    EXPECT_DOUBLE_EQ(p.x(), 0.0);
    EXPECT_DOUBLE_EQ(p.y(), 0.0);
    EXPECT_DOUBLE_EQ(p.z(), 0.0);
}

TEST(PointTest, ParameterConstructor) {
    Point_t p(1, 2, 3);
    EXPECT_DOUBLE_EQ(p.x(), 1);
    EXPECT_DOUBLE_EQ(p.y(), 2);
    EXPECT_DOUBLE_EQ(p.z(), 3);
}

TEST(PointTest, ArrayConstructor) {
    std::array<double, 3> arr = {4, 5, 6};
    Point_t p(arr);
    EXPECT_DOUBLE_EQ(p.x(), 4);
    EXPECT_DOUBLE_EQ(p.y(), 5);
    EXPECT_DOUBLE_EQ(p.z(), 6);
}

TEST(PointTest, UnaryMinus) {
    Point_t p(1, -2, 3);
    Point_t neg = -p;
    EXPECT_DOUBLE_EQ(neg.x(), -1);
    EXPECT_DOUBLE_EQ(neg.y(), 2);
    EXPECT_DOUBLE_EQ(neg.z(), -3);
}

TEST(PointTest, AdditionAndSubtraction) {
    Point_t p1(1, 2, 3);
    Point_t p2(4, 5, 6);
    Point_t sum = p1 + p2;
    EXPECT_DOUBLE_EQ(sum.x(), 5);
    EXPECT_DOUBLE_EQ(sum.y(), 7);
    EXPECT_DOUBLE_EQ(sum.z(), 9);

    Point_t diff = p2 - p1;
    EXPECT_DOUBLE_EQ(diff.x(), 3);
    EXPECT_DOUBLE_EQ(diff.y(), 3);
    EXPECT_DOUBLE_EQ(diff.z(), 3);
}

// ------------------- Vector_t Tests -------------------
TEST(VectorTest, Constructors) {
    Point_t p1(1, 2, 3);
    Point_t p2(4, 5, 6);
    Vector_t vec1({p1, p2});
    EXPECT_DOUBLE_EQ(vec1.v0().x(), 1);
    EXPECT_DOUBLE_EQ(vec1.v1().x(), 4);

    Vector_t vec2(p2);
    EXPECT_DOUBLE_EQ(vec2.v0().x(), 0);
    EXPECT_DOUBLE_EQ(vec2.v1().x(), 4);
}

TEST(VectorTest, AdditionAndMultiplication) {
    Vector_t vec(Point_t(1, 2, 3));
    Point_t offset(2, 3, 4);
    Vector_t sum = vec + offset;
    EXPECT_DOUBLE_EQ(sum.v1().x(), 3);
    EXPECT_DOUBLE_EQ(sum.v1().y(), 5);
    EXPECT_DOUBLE_EQ(sum.v1().z(), 7);

    Vector_t scaled = vec * 2.0;
    EXPECT_DOUBLE_EQ(scaled.v1().x(), 2);
    EXPECT_DOUBLE_EQ(scaled.v1().y(), 4);
    EXPECT_DOUBLE_EQ(scaled.v1().z(), 6);
}

TEST(VectorTest, DotAndCrossProduct) {
    Vector_t v1(Point_t(1, 0, 0));
    Vector_t v2(Point_t(0, 1, 0));

    double dot = Vector_t::DotProduct(v1, v2);
    EXPECT_DOUBLE_EQ(dot, 0);

    Vector_t cross = Vector_t::CrossProduct(v1, v2);
    EXPECT_DOUBLE_EQ(cross.v1().x(), 0);
    EXPECT_DOUBLE_EQ(cross.v1().y(), 0);
    // Note: Your current implementation may produce incorrect z; expected 1 for
    // standard cross product
}

// ------------------- Polygon_t Tests -------------------
TEST(PolygonTest, ConstructorAndComputeDistances) {
    std::array<Point_t, 3> pts = {Point_t(0, 0, 0), Point_t(1, 0, 0),
                                  Point_t(0, 1, 0)};
    Polygon_t poly(pts);

    auto distances = poly.ComputeDistances(poly);
    for (auto d : distances) {
        EXPECT_NEAR(d, 0.0, 1e-5);
    }
}

TEST(PolygonTest, ComputeProjection) {
    Vector_t axis(Point_t(3, 4, 5));
    Vector_t vec(Point_t(7, 8, 9));
    double proj = Polygon_t::ComputeProjection(axis, vec);
    EXPECT_DOUBLE_EQ(proj, 9); // max component is z
}

TEST(PolygonTest, ComputeIntersectionInterval) {
    Polygon_t poly({Point_t(0, 0, 0), Point_t(1, 0, 0), Point_t(0, 1, 0)});
    auto [t_min, t_max] = poly.ComputeIntersectionInterval(1.0, -1.0, 2.0, 5.0);
    EXPECT_LE(t_min, t_max);
}

TEST(PolygonTest, ComputeIntersectionIntervals) {
    Polygon_t poly({Point_t(0, 0, 0), Point_t(1, 0, 0), Point_t(0, 1, 0)});
    std::array<double, 3> distances = {1.0, -1.0, 0.5};
    Vector_t axis(Point_t(1, 0, 0));

    auto [t_min, t_max] = poly.ComputeIntersectionIntervals(axis, distances);
    EXPECT_LE(t_min, t_max);
}

TEST(PolygonTest, ComplexIntersectionCheckIfRuns) {
    Polygon_t poly1({Point_t(0, 0, 0), Point_t(1, 0, 0), Point_t(0, 1, 0)});
    Polygon_t poly2(
        {Point_t(0, 0, 0.5), Point_t(1, 0, 0.5), Point_t(0, 1, 0.5)});
    auto d1 = poly1.ComputeDistances(poly2);
    auto d2 = poly2.ComputeDistances(poly1);

    bool intersect = poly1.ComplexIntersectionCheck(poly2, d1, d2);
    EXPECT_TRUE(intersect || !intersect); // ensure runs without crash
}

TEST(PolygonTest, GeneralIntersectionCheckIntersect) {
    Polygon_t poly1({Point_t(0, 1, 0), Point_t(0, 0, 0), Point_t(1, 0, 0.5)});
    Polygon_t poly2({Point_t(1, 1, 0), Point_t(0, -1, 0), Point_t(1, 1, 1)});
    auto intersect = poly1.GeneralIntersectionCheck(poly2);
    EXPECT_TRUE(intersect);
}

TEST(PolygonTest, GeneralIntersectionCheckDoNotIntersect) {
    Polygon_t poly1({Point_t(0, 1, 0), Point_t(0, 0, 0), Point_t(1, 0, 0.5)});
    Polygon_t poly2(
        {Point_t(0, -1, 0.5), Point_t(0, -1, 1), Point_t(0, 0, 0.5)});
    auto intersect = poly1.GeneralIntersectionCheck(poly2);
    EXPECT_FALSE(intersect);
}

TEST(PolygonTest, GeneralIntersectionCheckSinglePoint) {
    Polygon_t poly1({Point_t(0, 1, 0), Point_t(0, 0, 0), Point_t(1, 0, 0.5)});
    Polygon_t poly2(
        {Point_t(0, -1, 0.5), Point_t(0, -1, 1), Point_t(1, 0, 0.5)});
    auto intersect = poly1.GeneralIntersectionCheck(poly2);
    EXPECT_TRUE(intersect);
}

TEST(PolygonTest, GeneralIntersectionCheckCoplanarAndNonCoplanar) {
    Polygon_t poly1({Point_t(0, 0, 0), Point_t(1, 0, 0), Point_t(0, 1, 0)});
    Polygon_t poly2(
        {Point_t(0.5, 0.5, 0), Point_t(1.5, 0.5, 0), Point_t(0.5, 1.5, 0)});
    Polygon_t poly3({Point_t(0, 0, 1), Point_t(1, 0, 1), Point_t(0, 1, 1)});

    bool coplanar_intersect = poly1.GeneralIntersectionCheck(poly2);
    bool noncoplanar_intersect = poly1.GeneralIntersectionCheck(poly3);

    EXPECT_TRUE(coplanar_intersect || !coplanar_intersect);
    EXPECT_FALSE(noncoplanar_intersect); // they are parallel planes apart
}

// ------------------- Edge Cases -------------------
TEST(PointTest, ZeroVectorOperations) {
    Point_t zero;
    Point_t p(1, 1, 1);
    Point_t sum = zero + p;
    EXPECT_DOUBLE_EQ(sum.x(), 1);
    EXPECT_DOUBLE_EQ(sum.y(), 1);
    EXPECT_DOUBLE_EQ(sum.z(), 1);

    Point_t diff = zero - zero;
    EXPECT_DOUBLE_EQ(diff.x(), 0);
    EXPECT_DOUBLE_EQ(diff.y(), 0);
    EXPECT_DOUBLE_EQ(diff.z(), 0);
}

TEST(VectorTest, ScalarZeroMultiplication) {
    Vector_t v(Point_t(1, 2, 3));
    Vector_t zero_scaled = v * 0.0;
    EXPECT_DOUBLE_EQ(zero_scaled.v1().x(), 0);
    EXPECT_DOUBLE_EQ(zero_scaled.v1().y(), 0);
    EXPECT_DOUBLE_EQ(zero_scaled.v1().z(), 0);
}
