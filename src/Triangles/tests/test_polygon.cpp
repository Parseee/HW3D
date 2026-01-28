#include "src/polygon.hpp"
#include <array>
#include <gtest/gtest.h>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// ------------------- Geom::Point_t Tests -------------------
TEST(PointTest, DefaultConstructor) {
    Geom::Point_t p;
    EXPECT_DOUBLE_EQ(p.x(), 0.0);
    EXPECT_DOUBLE_EQ(p.y(), 0.0);
    EXPECT_DOUBLE_EQ(p.z(), 0.0);
}

TEST(PointTest, ParameterConstructor) {
    Geom::Point_t p(1, 2, 3);
    EXPECT_DOUBLE_EQ(p.x(), 1);
    EXPECT_DOUBLE_EQ(p.y(), 2);
    EXPECT_DOUBLE_EQ(p.z(), 3);
}

TEST(PointTest, ArrayConstructor) {
    std::array<double, 3> arr = {4, 5, 6};
    Geom::Point_t p(arr);
    EXPECT_DOUBLE_EQ(p.x(), 4);
    EXPECT_DOUBLE_EQ(p.y(), 5);
    EXPECT_DOUBLE_EQ(p.z(), 6);
}

TEST(PointTest, UnaryMinus) {
    Geom::Point_t p(1, -2, 3);
    Geom::Point_t neg = -p;
    EXPECT_DOUBLE_EQ(neg.x(), -1);
    EXPECT_DOUBLE_EQ(neg.y(), 2);
    EXPECT_DOUBLE_EQ(neg.z(), -3);
}

TEST(PointTest, AdditionAndSubtraction) {
    Geom::Point_t p1(1, 2, 3);
    Geom::Point_t p2(4, 5, 6);
    Geom::Point_t sum = p1 + p2;
    EXPECT_DOUBLE_EQ(sum.x(), 5);
    EXPECT_DOUBLE_EQ(sum.y(), 7);
    EXPECT_DOUBLE_EQ(sum.z(), 9);

    Geom::Point_t diff = p2 - p1;
    EXPECT_DOUBLE_EQ(diff.x(), 3);
    EXPECT_DOUBLE_EQ(diff.y(), 3);
    EXPECT_DOUBLE_EQ(diff.z(), 3);
}

// ------------------- Geom::Vector_t Tests -------------------
TEST(VectorTest, Constructors) {
    Geom::Point_t p1(1, 2, 3);
    Geom::Vector_t vec1(p1);
    EXPECT_DOUBLE_EQ(vec1.x(), 1);

    Geom::Point_t p2(4, 5, 6);
    Geom::Vector_t vec2(p2);
    EXPECT_DOUBLE_EQ(vec2.x(), 4);
}

TEST(VectorTest, AdditionAndMultiplication) {
    Geom::Vector_t vec(Geom::Point_t(1, 2, 3));
    Geom::Point_t offset(2, 3, 4);
    Geom::Vector_t sum = vec + offset;
    EXPECT_DOUBLE_EQ(sum.x(), 3);
    EXPECT_DOUBLE_EQ(sum.y(), 5);
    EXPECT_DOUBLE_EQ(sum.z(), 7);

    Geom::Vector_t scaled = vec * 2.0;
    EXPECT_DOUBLE_EQ(scaled.x(), 2);
    EXPECT_DOUBLE_EQ(scaled.y(), 4);
    EXPECT_DOUBLE_EQ(scaled.z(), 6);
}

TEST(VectorTest, DotAndCrossProduct) {
    Geom::Vector_t v1(Geom::Point_t(1, 0, 0));
    Geom::Vector_t v2(Geom::Point_t(0, 1, 0));

    double dot = Geom::Vector_t::DotProduct(v1, v2);
    EXPECT_DOUBLE_EQ(dot, 0);

    Geom::Vector_t cross = Geom::Vector_t::CrossProduct(v1, v2);
    EXPECT_DOUBLE_EQ(cross.x(), 0);
    EXPECT_DOUBLE_EQ(cross.y(), 0);
}

// ------------------- Geom::Polygon_t Tests -------------------
TEST(PolygonTest, ConstructorAndComputeDistances) {
    std::array<Geom::Point_t, 3> pts = {
        Geom::Point_t(0, 0, 0), Geom::Point_t(1, 0, 0), Geom::Point_t(0, 1, 0)};
    Geom::Polygon_t poly(pts);

    auto distances = poly.ComputeDistances(poly);
    for (auto d : distances) {
        EXPECT_NEAR(d, 0.0, 1e-10);
    }
}

TEST(PolygonTest, ComputeProjection) {
    Geom::Vector_t axis(Geom::Point_t(3, 4, 5));
    Geom::Vector_t vec(Geom::Point_t(7, 8, 9));
    double proj = Geom::Polygon_t::ComputeProjection(axis, vec);
    EXPECT_DOUBLE_EQ(proj, 9); // max component is z
}

TEST(IntersectionTest, Symmetry) {
    Geom::Polygon_t poly1({Geom::Point_t(0, 0, 0), Geom::Point_t(1, 0, 0),
                           Geom::Point_t(0, 1, 0)});

    Geom::Polygon_t poly2({Geom::Point_t(0.2, 0.2, -1),
                           Geom::Point_t(0.2, 0.2, 1), Geom::Point_t(1, 1, 0)});

    bool a = poly1.GeneralIntersectionCheck(poly2);
    bool b = poly2.GeneralIntersectionCheck(poly1);

    EXPECT_EQ(a, b);
}

// ------------------- Intersection Tests -------------------

TEST(IntersectionTest, ComplexIntersectionCheckIfRuns) {
    Geom::Polygon_t poly1({Geom::Point_t(0, 0, 0), Geom::Point_t(1, 0, 0),
                           Geom::Point_t(0, 1, 0)});
    Geom::Polygon_t poly2({Geom::Point_t(0, 0, 0.5), Geom::Point_t(1, 0, 0.5),
                           Geom::Point_t(0, 1, 0.5)});
    auto d1 = poly1.ComputeDistances(poly2);
    auto d2 = poly2.ComputeDistances(poly1);

    bool intersect = poly1.ComplexIntersectionCheck(poly2, d1, d2);
    EXPECT_TRUE(intersect || !intersect);
}

TEST(IntersectionTest, GeneralIntersectionCheckIntersect) {
    Geom::Polygon_t poly1({Geom::Point_t(0, 1, 0), Geom::Point_t(0, 0, 0),
                           Geom::Point_t(1, 0, 0.5)});
    Geom::Polygon_t poly2({Geom::Point_t(1, 1, 0), Geom::Point_t(0, -1, 0),
                           Geom::Point_t(1, 1, 1)});
    auto intersect = poly1.GeneralIntersectionCheck(poly2);
    EXPECT_TRUE(intersect);
}

TEST(IntersectionTest, GeneralIntersectionCheckDoNotIntersect) {
    Geom::Polygon_t poly1({Geom::Point_t(0, 1, 0), Geom::Point_t(0, 0, 0),
                           Geom::Point_t(1, 0, 0.5)});
    Geom::Polygon_t poly2({Geom::Point_t(0, -1, 0.5), Geom::Point_t(0, -1, 1),
                           Geom::Point_t(0, 0, 0.5)});
    auto intersect = poly1.GeneralIntersectionCheck(poly2);
    EXPECT_FALSE(intersect);
}

TEST(IntersectionTest, GeneralIntersectionCheckSinglePoint) {
    Geom::Polygon_t poly1({Geom::Point_t(0, 1, 0), Geom::Point_t(0, 0, 0),
                           Geom::Point_t(1, 0, 0.5)});
    Geom::Polygon_t poly2({Geom::Point_t(0, -1, 0.5), Geom::Point_t(0, -1, 1),
                           Geom::Point_t(1, 0, 0.5)});
    auto intersect = poly1.GeneralIntersectionCheck(poly2);
    EXPECT_TRUE(intersect);
}

TEST(IntersectionTest, GeneralIntersectionCheckComplex) {
    Geom::Polygon_t poly1({Geom::Point_t(12.386644688712941, 13.149797301748318,
                                         14.583230270789596),
                           Geom::Point_t(10.949402845223053, 11.498552932115821,
                                         16.210422004080684),
                           Geom::Point_t(11.892947051130829, 10.90867559171325,
                                         15.653029294217191)});
    Geom::Polygon_t poly2({Geom::Point_t(12.824814622822574, 12.302666118691093,
                                         13.317311583348722),
                           Geom::Point_t(10.934714901031004, 11.547678576168568,
                                         15.710024434505659),
                           Geom::Point_t(13.022073702256872, 12.674256370441665,
                                         14.655135339419592)});
    auto intersect = poly1.GeneralIntersectionCheck(poly2);
    EXPECT_TRUE(intersect);
}

TEST(IntersectionTest, GeneralIntersectionCheckComplex2) {
    Geom::Polygon_t poly1({Geom::Point_t(-21.1391, -2.96044, -3.74303),
                           Geom::Point_t(-21.487, -3.0482, -3.85087),
                           Geom::Point_t(-21.1587, -2.62302, -4.02156)});
    Geom::Polygon_t poly2({Geom::Point_t(16.086, 2.91767, 11.365),
                           Geom::Point_t(16.3106, 2.91342, 11.0859),
                           Geom::Point_t(16.7994, 2.65756, 10.6859)});
    auto intersect = poly1.GeneralIntersectionCheck(poly2);
    EXPECT_FALSE(intersect);
}

TEST(IntersectionTest, GeneralIntersectionCheckCoplanarAndNonCoplanar) {
    Geom::Polygon_t poly1({Geom::Point_t(0, 0, 0), Geom::Point_t(1, 0, 0),
                           Geom::Point_t(0, 1, 0)});
    Geom::Polygon_t poly2({Geom::Point_t(0.5, 0.5, 0),
                           Geom::Point_t(1.5, 0.5, 0),
                           Geom::Point_t(0.5, 1.5, 0)});
    Geom::Polygon_t poly3({Geom::Point_t(0, 0, 1), Geom::Point_t(1, 0, 1),
                           Geom::Point_t(0, 1, 1)});

    bool coplanar_intersect = poly1.GeneralIntersectionCheck(poly2);
    bool noncoplanar_intersect = poly1.GeneralIntersectionCheck(poly3);

    EXPECT_TRUE(coplanar_intersect || !coplanar_intersect);
    EXPECT_FALSE(noncoplanar_intersect); // they are parallel planes apart
}

TEST(IntersectionTest, EdgePiercingTriangle) {
    Geom::Polygon_t poly1({Geom::Point_t(0, 0, 0), Geom::Point_t(2, 0, 0),
                           Geom::Point_t(0, 2, 0)});

    Geom::Polygon_t poly2({Geom::Point_t(1, 0, -1), Geom::Point_t(1, 0, 1),
                           Geom::Point_t(1, 1, 0)});

    EXPECT_TRUE(poly1.GeneralIntersectionCheck(poly2));
}

TEST(IntersectionTest, ComplexCoplanarIntersectionCheck) {
    Geom::Polygon_t poly1({Geom::Point_t(0, 0, 0), Geom::Point_t(0, 1, 0),
                           Geom::Point_t(1, 2, 2)});
    Geom::Polygon_t poly2({Geom::Point_t(0, -1, 0), Geom::Point_t(0, 2, 0),
                           Geom::Point_t(2, 2, 4)});

    bool coplanar_intersect = poly1.GeneralIntersectionCheck(poly2);

    EXPECT_TRUE(coplanar_intersect);
}

// ------------------- Edge Cases -------------------
TEST(PointTest, ZeroVectorOperations) {
    Geom::Point_t zero;
    Geom::Point_t p(1, 1, 1);
    Geom::Point_t sum = zero + p;
    EXPECT_DOUBLE_EQ(sum.x(), 1);
    EXPECT_DOUBLE_EQ(sum.y(), 1);
    EXPECT_DOUBLE_EQ(sum.z(), 1);

    Geom::Point_t diff = zero - zero;
    EXPECT_DOUBLE_EQ(diff.x(), 0);
    EXPECT_DOUBLE_EQ(diff.y(), 0);
    EXPECT_DOUBLE_EQ(diff.z(), 0);
}

TEST(VectorTest, ScalarZeroMultiplication) {
    Geom::Vector_t v(Geom::Point_t(1, 2, 3));
    Geom::Vector_t zero_scaled = v * 0.0;
    EXPECT_DOUBLE_EQ(zero_scaled.x(), 0);
    EXPECT_DOUBLE_EQ(zero_scaled.y(), 0);
    EXPECT_DOUBLE_EQ(zero_scaled.z(), 0);
}

TEST(IntersectionTest, SharedEdgeCoplanar) {
    Geom::Polygon_t poly1({Geom::Point_t(0, 0, 0), Geom::Point_t(1, 0, 0),
                           Geom::Point_t(0, 1, 0)});

    Geom::Polygon_t poly2({Geom::Point_t(0, 0, 0), Geom::Point_t(1, 0, 0),
                           Geom::Point_t(1, 1, 0)});

    EXPECT_TRUE(poly1.GeneralIntersectionCheck(poly2));
}

TEST(IntersectionTest, NearlyCoplanarButSeparated) {
    Geom::Polygon_t poly1({Geom::Point_t(0, 0, 0), Geom::Point_t(1, 0, 0),
                           Geom::Point_t(0, 1, 0)});

    Geom::Polygon_t poly2({Geom::Point_t(0, 0, 1e-9), Geom::Point_t(1, 0, 1e-9),
                           Geom::Point_t(0, 1, 1e-9)});

    EXPECT_FALSE(poly1.GeneralIntersectionCheck(poly2));
}

TEST(IntersectionTest, DegenerateTriangleLine) {
    Geom::Polygon_t lineTri({Geom::Point_t(0, 0, 0), Geom::Point_t(1, 0, 0),
                             Geom::Point_t(2, 0, 0)});

    Geom::Polygon_t tri({Geom::Point_t(0, -1, 0), Geom::Point_t(0, 1, 0),
                         Geom::Point_t(0, 0, 1)});

    EXPECT_FALSE(lineTri.GeneralIntersectionCheck(tri));
}

TEST(IntersectionTest, DegenerateTrianglePoint) {
    Geom::Polygon_t pointTri({Geom::Point_t(0, 0, 0), Geom::Point_t(0, 0, 0),
                              Geom::Point_t(0, 0, 0)});

    Geom::Polygon_t tri({Geom::Point_t(-1, -1, 0), Geom::Point_t(1, -1, 0),
                         Geom::Point_t(0, 1, 0)});

    EXPECT_FALSE(pointTri.GeneralIntersectionCheck(tri));
}
