#include <gtest/gtest.h>
#include "../src/GeometryLibrary.hpp" // Include the header where Fracture and readFractures are defined
#include "../src/input-output.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


TEST(FRACTURES, readFractures) {
    const std::string path = "./DFN/FR3_data.txt";

    std::vector<Fracture> fractures;
    double tol=10*numeric_limits<double>::epsilon();
    unsigned int numFractures = readFractures(path, fractures, tol);

    EXPECT_EQ(numFractures, 3);
    EXPECT_EQ(fractures.size(), 3);
    EXPECT_EQ(fractures[0].idFrac, 0);
    EXPECT_EQ(fractures[0].vertices.size(), 4);

    EXPECT_TRUE(areVectorsEqual(fractures[0].vertices[0],Vector3d(0,0,0),tol));
    EXPECT_TRUE(areVectorsEqual(fractures[0].vertices[1],Vector3d(1,0,0),tol));
    EXPECT_TRUE(areVectorsEqual(fractures[0].vertices[2],Vector3d(1,1,0),tol));
    EXPECT_TRUE(areVectorsEqual(fractures[0].vertices[3],Vector3d(0,1,0),tol));

}

TEST(FRACTURES, FindNormal) {
    double tol=10*numeric_limits<double>::epsilon();
    Vector3d p1(0, 0, 0);
    Vector3d p2(1, 0, 0);
    Vector3d p3(1, 1, 0);

    Vector3d expected(0, 0, 1);
    Vector3d result = findNormal(p1, p2, p3);

    EXPECT_TRUE(areVectorsEqual(expected, result, tol));
}

TEST(FRACTURES, FindNormalSamePoints) {
    double tol=10*numeric_limits<double>::epsilon();
    Vector3d p1(1, 1, 1);
    Vector3d p2(1, 1, 1);
    Vector3d p3(1, 1, 1);

    Vector3d expected(0, 0, 0);
    Vector3d result = findNormal(p1, p2, p3);

    EXPECT_TRUE(areVectorsEqual(expected, result, tol));
}

TEST(FRACTURES, FindNormalPointsOnSameLine) {
    double tol=10*numeric_limits<double>::epsilon();
    Vector3d p1(1, 1, 1);
    Vector3d p2(2, 2, 2);
    Vector3d p3(3, 3, 3);

    Vector3d expected(0, 0, 0);
    Vector3d result = findNormal(p1, p2, p3);

    EXPECT_TRUE(areVectorsEqual(expected, result, tol));
}

TEST(FRACTURES, FindLyingPlaneTest) {
    // Tolleranza
    double tol = 1e-6;

    // Test per il piano XY
    Vector3d n1 = {0, 0, 1}; // Perpendicolare al piano XY
    EXPECT_EQ(findLyingPlane(n1, tol), 1);

    // Test per il piano XZ
    Vector3d n2 = {0, 1, 0}; // Perpendicolare al piano XZ
    EXPECT_EQ(findLyingPlane(n2, tol), 2);

    // Test per il piano YZ
    Vector3d n3 = {1, 0, 0}; // Perpendicolare al piano YZ
    EXPECT_EQ(findLyingPlane(n3, tol), 3);

    // Test per nessun piano
    Vector3d n4 = {1, 1, 1}; // Non perpendicolare a nessun piano
    EXPECT_EQ(findLyingPlane(n4, tol), 0);
}

// CheckFractureEdges

// areVectorsEqual
// Test per vettori esattamente uguali
TEST(AreVectorsEqualTest, VectorsExactlyEqual) {
    Vector3d v1(1.0, 2.0, 3.0);
    Vector3d v2(1.0, 2.0, 3.0);
    double tol = 1e-6;

    EXPECT_TRUE(areVectorsEqual(v1, v2, tol));
}

// Test per vettori uguali entro la tolleranza
TEST(AreVectorsEqualTest, VectorsEqualWithinTolerance) {
    Vector3d v1(1.0, 2.0, 3.0);
    Vector3d v2(1.0 + 1e-7, 2.0 - 1e-7, 3.0 + 1e-7);
    double tol = 1e-6;

    EXPECT_TRUE(areVectorsEqual(v1, v2, tol));
}

// Test per vettori non uguali oltre la tolleranza
TEST(AreVectorsEqualTest, VectorsNotEqualBeyondTolerance) {
    Vector3d v1(1.0, 2.0, 3.0);
    Vector3d v2(1.0 + 1e-5, 2.0 - 1e-5, 3.0 + 1e-5);
    double tol = 1e-6;

    EXPECT_FALSE(areVectorsEqual(v1, v2, tol));
}

// Test per vettori molto distanti
TEST(AreVectorsEqualTest, VectorsVeryDifferent) {
    Vector3d v1(1.0, 2.0, 3.0);
    Vector3d v2(10.0, 20.0, 30.0);
    double tol = 1e-6;

    EXPECT_FALSE(areVectorsEqual(v1, v2, tol));
}

// computeBoundingSphere
// Test per un singolo punto
TEST(ComputeBoundingSphereTest, SinglePoint) {
    vector<Vector3d> vertices = {Vector3d(1.0, 1.0, 1.0)};
    BoundingSphere sphere = computeBoundingSphere(vertices);

    EXPECT_EQ(sphere.centroid, Vector3d(1.0, 1.0, 1.0));
    EXPECT_DOUBLE_EQ(sphere.radius, 0.0);
}

// Test per due punti
TEST(ComputeBoundingSphereTest, TwoPoints) {
    vector<Vector3d> vertices = {Vector3d(1.0, 1.0, 1.0), Vector3d(-1.0, -1.0, -1.0)};
    BoundingSphere sphere = computeBoundingSphere(vertices);

    EXPECT_EQ(sphere.centroid, Vector3d(0.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(sphere.radius, sqrt(3));
}

// Test per quattro punti simmetrici
TEST(ComputeBoundingSphereTest, FourSymmetricPoints) {
    vector<Vector3d> vertices = {
        Vector3d(1.0, 0.0, 0.0),
        Vector3d(-1.0, 0.0, 0.0),
        Vector3d(0.0, 1.0, 0.0),
        Vector3d(0.0, -1.0, 0.0)
    };
    BoundingSphere sphere = computeBoundingSphere(vertices);

    EXPECT_EQ(sphere.centroid, Vector3d(0.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(sphere.radius, 1.0);
}

// Test per punti casuali
TEST(ComputeBoundingSphereTest, RandomPoints) {
    vector<Vector3d> vertices = {
        Vector3d(1.0, 2.0, 3.0),
        Vector3d(4.0, 5.0, 6.0),
        Vector3d(-1.0, -2.0, -3.0),
        Vector3d(-4.0, -5.0, -6.0)
    };
    BoundingSphere sphere = computeBoundingSphere(vertices);

    // Il centroide dovrebbe essere il punto medio
    EXPECT_NEAR(sphere.centroid.x(), 0.0, 1e-6);
    EXPECT_NEAR(sphere.centroid.y(), 0.0, 1e-6);
    EXPECT_NEAR(sphere.centroid.z(), 0.0, 1e-6);

    // Il raggio dovrebbe essere la distanza massima dal centroide
    double expectedRadius = (Vector3d(-4.0, -5.0, -6.0) - Vector3d(0.0, 0.0, 0.0)).norm();
    EXPECT_NEAR(sphere.radius, expectedRadius, 1e-6);
}

// spheresIntersect
// Test per sfere che si intersecano
TEST(SpheresIntersectTest, SpheresIntersect) {
    BoundingSphere sphere1 = {Vector3d(0.0, 0.0, 0.0), 1.0};
    BoundingSphere sphere2 = {Vector3d(1.0, 0.0, 0.0), 1.0};

    EXPECT_TRUE(spheresIntersect(sphere1, sphere2));
}

// Test per sfere che non si intersecano
TEST(SpheresIntersectTest, SpheresDoNotIntersect) {
    BoundingSphere sphere1 = {Vector3d(0.0, 0.0, 0.0), 1.0};
    BoundingSphere sphere2 = {Vector3d(3.0, 0.0, 0.0), 1.0};

    EXPECT_FALSE(spheresIntersect(sphere1, sphere2));
}

// Test per sfere che si toccano esattamente in un punto
TEST(SpheresIntersectTest, SpheresTouchAtOnePoint) {
    BoundingSphere sphere1 = {Vector3d(0.0, 0.0, 0.0), 1.0};
    BoundingSphere sphere2 = {Vector3d(2.0, 0.0, 0.0), 1.0};

    EXPECT_TRUE(spheresIntersect(sphere1, sphere2));
}

// Test per sfere coincidenti
TEST(SpheresIntersectTest, SpheresAreCoincident) {
    BoundingSphere sphere1 = {Vector3d(0.0, 0.0, 0.0), 1.0};
    BoundingSphere sphere2 = {Vector3d(0.0, 0.0, 0.0), 1.0};

    EXPECT_TRUE(spheresIntersect(sphere1, sphere2));
}

// Test per sfere con raggio zero
TEST(SpheresIntersectTest, SpheresWithZeroRadius) {
    BoundingSphere sphere1 = {Vector3d(0.0, 0.0, 0.0), 0.0};
    BoundingSphere sphere2 = {Vector3d(0.0, 0.0, 0.0), 0.0};

    EXPECT_TRUE(spheresIntersect(sphere1, sphere2));

    BoundingSphere sphere3 = {Vector3d(1.0, 0.0, 0.0), 0.0};

    EXPECT_FALSE(spheresIntersect(sphere1, sphere3));
}





