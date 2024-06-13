#include <gtest/gtest.h>
#include "../src/GeometryLibrary.hpp" // Include the header where Fracture and readFractures are defined
#include "../src/input-output.hpp"
#include "../src/Utils.hpp"
#include "../src/PolygonalMesh.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


TEST(findNormal, readFractures) {
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

TEST(findNormal, baseCase) {
    double tol=10*numeric_limits<double>::epsilon();
    Vector3d p1(0, 0, 0);
    Vector3d p2(1, 0, 0);
    Vector3d p3(1, 1, 0);

    Vector3d expected(0, 0, 1);
    Vector3d result = findNormal(p1, p2, p3);

    EXPECT_TRUE(areVectorsEqual(expected, result, tol));
}

TEST(findNormal, FindNormalSamePoints) {
    double tol=10*numeric_limits<double>::epsilon();
    Vector3d p1(1, 1, 1);
    Vector3d p2(1, 1, 1);
    Vector3d p3(1, 1, 1);

    Vector3d expected(0, 0, 0);
    Vector3d result = findNormal(p1, p2, p3);

    EXPECT_TRUE(areVectorsEqual(expected, result, tol));
}

TEST(findNormal, FindNormalPointsOnSameLine) {
    double tol=10*numeric_limits<double>::epsilon();
    Vector3d p1(1, 1, 1);
    Vector3d p2(2, 2, 2);
    Vector3d p3(3, 3, 3);

    Vector3d expected(0, 0, 0);
    Vector3d result = findNormal(p1, p2, p3);

    EXPECT_TRUE(areVectorsEqual(expected, result, tol));
}

TEST(findLyingPlane, baseCase) {
    // Tolleranza
    double tol=10*numeric_limits<double>::epsilon();

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
TEST(AreVectorsEqual, VectorsExactlyEqual) {
    Vector3d v1(1, 2, 3);
    Vector3d v2(1, 2, 3);
    double tol=10*numeric_limits<double>::epsilon();

    EXPECT_TRUE(areVectorsEqual(v1, v2, tol));
}

// Test per vettori uguali entro la tolleranza
TEST(AreVectorsEqual, VectorsEqualWithinTolerance) {
    Vector3d v1(1, 2, 3);
    Vector3d v2(1 + 1e-7, 2 - 1e-7, 3 + 1e-7);
    double tol=1e-6;
    EXPECT_TRUE(areVectorsEqual(v1, v2, tol));
}

// Test per vettori non uguali oltre la tolleranza
TEST(AreVectorsEqual, VectorsNotEqualBeyondTolerance) {
    Vector3d v1(1, 2, 3);
    Vector3d v2(1 + 1e-5, 2 - 1e-5, 3 + 1e-5);
    double tol=1e-6;

    EXPECT_FALSE(areVectorsEqual(v1, v2, tol));
}

// Test per vettori molto distanti
TEST(AreVectorsEqual, VectorsVeryDifferent) {
    Vector3d v1(1, 2, 3);
    Vector3d v2(10, 20, 30);
    double tol=10*numeric_limits<double>::epsilon();

    EXPECT_FALSE(areVectorsEqual(v1, v2, tol));
}

// computeBoundingSphere
// Test per un singolo punto
TEST(ComputeBoundingSphere, SinglePoint) {
    vector<Vector3d> vertices = {Vector3d(1, 1, 1)};
    BoundingSphere sphere = computeBoundingSphere(vertices);
    double tol=10*numeric_limits<double>::epsilon();

    EXPECT_TRUE(areVectorsEqual(sphere.centroid, Vector3d(0, 0, 0), tol));
    EXPECT_DOUBLE_EQ(sphere.radius, 0);
}

// Test per due punti
TEST(ComputeBoundingSphere, TwoPoints) {
    vector<Vector3d> vertices = {Vector3d(1, 1, 1), Vector3d(-1, -1, -1)};
    BoundingSphere sphere = computeBoundingSphere(vertices);
    double tol=10*numeric_limits<double>::epsilon();

    EXPECT_TRUE(areVectorsEqual(sphere.centroid, Vector3d(0, 0, 0), tol));
    EXPECT_DOUBLE_EQ(sphere.radius, 0);
}

// Test per quattro punti simmetrici
TEST(ComputeBoundingSphere, FourSymmetricPoints) {
    vector<Vector3d> vertices = {
        Vector3d(1, 0, 0),
        Vector3d(-1, 0, 0),
        Vector3d(0, 1, 0),
        Vector3d(0, -1, 0)
    };
    BoundingSphere sphere = computeBoundingSphere(vertices);

    double tol=10*numeric_limits<double>::epsilon();

    EXPECT_TRUE(areVectorsEqual(sphere.centroid, Vector3d(0, 0, 0), tol));
    EXPECT_DOUBLE_EQ(sphere.radius, 1);
}

// Test per punti casuali
TEST(ComputeBoundingSphere, RandomPoints) {
    vector<Vector3d> vertices = {
        Vector3d(1, 2, 3),
        Vector3d(4, 5, 6),
        Vector3d(-1, -2, -3),
        Vector3d(-4, -5, -6)
    };
    BoundingSphere sphere = computeBoundingSphere(vertices);

    // Il centroide dovrebbe essere il punto medio
    EXPECT_NEAR(sphere.centroid.x(), 0, 1e-6);
    EXPECT_NEAR(sphere.centroid.y(), 0, 1e-6);
    EXPECT_NEAR(sphere.centroid.z(), 0, 1e-6);

    // Il raggio dovrebbe essere la distanza massima dal centroide
    double expectedRadius = (Vector3d(-4, -5, -6) - Vector3d(0, 0, 0)).norm();
    EXPECT_NEAR(sphere.radius, expectedRadius, 1e-6);
}

// spheresIntersect
// Test per sfere che si intersecano
TEST(SpheresIntersect, baseCase) {
    BoundingSphere sphere1 = {Vector3d(0, 0, 0), 1};
    BoundingSphere sphere2 = {Vector3d(1, 0, 0), 1};

    EXPECT_TRUE(spheresIntersect(sphere1, sphere2));
}

// Test per sfere che non si intersecano
TEST(SpheresIntersect, SpheresDoNotIntersect) {
    BoundingSphere sphere1 = {Vector3d(0, 0, 0), 1};
    BoundingSphere sphere2 = {Vector3d(3, 0, 0), 1};

    EXPECT_FALSE(spheresIntersect(sphere1, sphere2));
}

// Test per sfere che si toccano esattamente in un punto
TEST(SpheresIntersect, SpheresTouchAtOnePoint) {
    BoundingSphere sphere1 = {Vector3d(0, 0, 0), 1};
    BoundingSphere sphere2 = {Vector3d(2, 0, 0), 1};

    EXPECT_TRUE(spheresIntersect(sphere1, sphere2));
}

// Test per sfere coincidenti
TEST(SpheresIntersect, SpheresAreCoincident) {
    BoundingSphere sphere1 = {Vector3d(0, 0, 0), 1};
    BoundingSphere sphere2 = {Vector3d(0, 0, 0), 1};

    EXPECT_TRUE(spheresIntersect(sphere1, sphere2));
}

// Test per sfere con raggio zero
TEST(SpheresIntersect, SpheresWithZeroRadius) {
    BoundingSphere sphere1 = {Vector3d(0, 0, 0), 0};
    BoundingSphere sphere2 = {Vector3d(0, 0, 0), 0};

    EXPECT_TRUE(spheresIntersect(sphere1, sphere2));

    BoundingSphere sphere3 = {Vector3d(1, 0, 0), 0};

    EXPECT_FALSE(spheresIntersect(sphere1, sphere3));
}

// projectOntoXY
// Test per un punto generico
TEST(ProjectOntoXY, GenericPoint) {
    Vector3d point(1, 2, 3);
    Vector2d result = projectOntoXY(point);
    EXPECT_EQ(result, Vector2d(1, 2));
}

// Test per il punto origine
TEST(ProjectOntoXY, OriginPoint) {
    Vector3d point(0, 0, 0);
    Vector2d result = projectOntoXY(point);
    EXPECT_EQ(result, Vector2d(0, 0));
}

// Test per un punto con coordinate negative
TEST(ProjectOntoXY, NegativeCoordinates) {
    Vector3d point(-1, -2, -3);
    Vector2d result = projectOntoXY(point);
    EXPECT_EQ(result, Vector2d(-1, -2));
}

// Test per un punto con z non nullo
TEST(ProjectOntoXY, NonZeroZCoordinate) {
    Vector3d point(1, 2, 5);
    Vector2d result = projectOntoXY(point);
    EXPECT_EQ(result, Vector2d(1, 2));
}

// Test per grandi valori
TEST(ProjectOntoXY, LargeValues) {
    Vector3d point(1e6, 2e6, 3e6);
    Vector2d result = projectOntoXY(point);
    EXPECT_EQ(result, Vector2d(1e6, 2e6));
}

// projectOntoXZ
// Test per un punto generico
TEST(ProjectOntoXZ, GenericPoint) {
    Vector3d point(1, 2, 3);
    Vector2d result = projectOntoXZ(point);
    EXPECT_EQ(result, Vector2d(1, 3));
}

// Test per il punto origine
TEST(ProjectOntoXZ, OriginPoint) {
    Vector3d point(0, 0, 0);
    Vector2d result = projectOntoXZ(point);
    EXPECT_EQ(result, Vector2d(0, 0));
}

// Test per un punto con coordinate negative
TEST(ProjectOntoXZ, NegativeCoordinates) {
    Vector3d point(-1, -2, -3);
    Vector2d result = projectOntoXZ(point);
    EXPECT_EQ(result, Vector2d(-1, -3));
}

// Test per un punto con y non nullo
TEST(ProjectOntoXZ, NonZeroYCoordinate) {
    Vector3d point(1, 5, 2);
    Vector2d result = projectOntoXZ(point);
    EXPECT_EQ(result, Vector2d(1, 2));
}

// Test per grandi valori
TEST(ProjectOntoXZ, LargeValues) {
    Vector3d point(1e6, 2e6, 3e6);
    Vector2d result = projectOntoXZ(point);
    EXPECT_EQ(result, Vector2d(1e6, 3e6));
}

// projectOntoYZ
// Test per un punto generico
TEST(ProjectOntoYZ, GenericPoint) {
    Vector3d point(1, 2, 3);
    Vector2d result = projectOntoYZ(point);
    EXPECT_EQ(result, Vector2d(2, 3));
}

// Test per il punto origine
TEST(ProjectOntoYZ, OriginPoint) {
    Vector3d point(0, 0, 0);
    Vector2d result = projectOntoYZ(point);
    EXPECT_EQ(result, Vector2d(0, 0));
}

// Test per un punto con coordinate negative
TEST(ProjectOntoYZ, NegativeCoordinates) {
    Vector3d point(-1, -2, -3);
    Vector2d result = projectOntoYZ(point);
    EXPECT_EQ(result, Vector2d(-2, -3));
}

// Test per un punto con x non nullo
TEST(ProjectOntoYZ, NonZeroXCoordinate) {
    Vector3d point(5, 1, 2);
    Vector2d result = projectOntoYZ(point);
    EXPECT_EQ(result, Vector2d(1, 2));
}

// Test per grandi valori
TEST(ProjectOntoYZ, LargeValues) {
    Vector3d point(1e6, 2e6, 3e6);
    Vector2d result = projectOntoYZ(point);
    EXPECT_EQ(result, Vector2d(2e6, 3e6));
}

// projectIntersection

// projectVertices;

// Test per proiezione su piano XY
TEST(ProjectVertices, ProjectOntoXY) {
    Fracture F;
    F.lyingPlane = 1; // Plane 1 represents XY plane
    F.vertices.push_back(Vector3d(1, 2, 3));
    F.vertices.push_back(Vector3d(4, 5, 6));
    vector<Vector2d> result;
    projectVertices(result, F);
    ASSERT_EQ(result.size(), 2);
    EXPECT_EQ(result[0], Vector2d(1, 2));
    EXPECT_EQ(result[1], Vector2d(4, 5));
}

//  per proiezione su piano XZ
TEST(ProjectVertices, ProjectOntoXZ) {
    Fracture F;
    F.lyingPlane = 2; // Plane 2 represents XZ plane
    F.vertices.push_back(Vector3d(1, 2, 3));
    F.vertices.push_back(Vector3d(4, 5, 6));
    vector<Vector2d> result;
    projectVertices(result, F);
    ASSERT_EQ(result.size(), 2);
    EXPECT_EQ(result[0], Vector2d(1, 3));
    EXPECT_EQ(result[1], Vector2d(4, 6));
}

// Test per proiezione su piano YZ
TEST(ProjectVertices, ProjectOntoYZ) {
    Fracture F;
    F.lyingPlane = 3; // Plane 3 represents YZ plane
    F.vertices.push_back(Vector3d(1, 2, 3));
    F.vertices.push_back(Vector3d(4, 5, 6));
    vector<Vector2d> result;
    projectVertices(result, F);
    ASSERT_EQ(result.size(), 2);
    EXPECT_EQ(result[0], Vector2d(2, 3));
    EXPECT_EQ(result[1], Vector2d(5, 6));
}

//  isPointOn2DSegment(
// Test per un punto all'interno del segmento
TEST(PointOn2DSegment, PointInsideSegment) {
    Vector2d point(1, 1);
    Vector2d s1(0, 0);
    Vector2d s2(2, 2);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(isPointOn2DSegment(point, s1, s2, tol));
}

// Test per un punto esterno al segmento
TEST(PointOn2DSegment, PointOutsideSegment) {
    Vector2d point(3, 3);
    Vector2d s1(0, 0);
    Vector2d s2(2, 2);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_FALSE(isPointOn2DSegment(point, s1, s2, tol));
}

// Test per un punto coincidente con un'estremità del segmento
TEST(PointOn2DSegment, PointCoincideWithEndpoint) {
    Vector2d point(0, 0);
    Vector2d s1(0, 0);
    Vector2d s2(2, 2);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(isPointOn2DSegment(point, s1, s2, tol));
}

// Test per un segmento verticale
TEST(PointOn2DSegment, VerticalSegment) {
    Vector2d point(1, 1);
    Vector2d s1(1, 0);
    Vector2d s2(1, 2);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(isPointOn2DSegment(point, s1, s2, tol));
}

// isPointOn3DSegment
// Test per un punto all'interno del segmento
TEST(PointOn3DSegment, PointInsideSegment) {
    Vector3d point(1, 1, 1);
    Vector3d s1(0, 0, 0);
    Vector3d s2(2, 2, 2);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(isPointOn3DSegment(point, s1, s2, tol));
}

// Test per un punto esterno al segmento
TEST(PointOn3DSegment, PointOutsideSegment) {
    Vector3d point(3, 3, 3);
    Vector3d s1(0, 0, 0);
    Vector3d s2(2, 2, 2);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_FALSE(isPointOn3DSegment(point, s1, s2, tol));
}

// Test per un punto coincidente con un'estremità del segmento
TEST(PointOn3DSegment, PointCoincideWithEndpoint) {
    Vector3d point(0, 0, 0);
    Vector3d s1(0, 0, 0);
    Vector3d s2(2, 2, 2);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(isPointOn3DSegment(point, s1, s2, tol));
}

// Test per un segmento verticale
TEST(PointOn3DSegment, VerticalSegment) {
    Vector3d point(1, 1, 1);
    Vector3d s1(1, 0, 1);
    Vector3d s2(1, 2, 1);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(isPointOn3DSegment(point, s1, s2, tol));
}

// isPointIn2DPolygon
// Test per un punto all'interno del poligono
TEST(PointIn2DPolygon, PointInsidePolygon) {
    vector<Vector2d> polygon;
    polygon.push_back(Vector2d(1, 1));
    polygon.push_back(Vector2d(2, 1));
    polygon.push_back(Vector2d(2, 2));
    polygon.push_back(Vector2d(1, 2));
    Vector2d point(1.5, 1.5);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(isPointIn2DPolygon(point, polygon, tol));
}

// Test per un punto all'esterno del poligono
TEST(PointIn2DPolygon, PointOutsidePolygon) {
    vector<Vector2d> polygon;
    polygon.push_back(Vector2d(1, 1));
    polygon.push_back(Vector2d(2, 1));
    polygon.push_back(Vector2d(2, 2));
    polygon.push_back(Vector2d(1, 2));
    Vector2d point(0.5, 0.5);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_FALSE(isPointIn2DPolygon(point, polygon, tol));
}

// Test per un punto sul bordo del poligono
TEST(PointIn2DPolygon, PointOnPolygonEdge) {
    vector<Vector2d> polygon;
    polygon.push_back(Vector2d(1, 1));
    polygon.push_back(Vector2d(2, 1));
    polygon.push_back(Vector2d(2, 2));
    polygon.push_back(Vector2d(1, 2));
    Vector2d point(1, 1.5);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(isPointIn2DPolygon(point, polygon, tol));
}

// Test per un punto sul vertice del poligono
TEST(PointIn2DPolygon, PointOnPolygonVertex) {
    vector<Vector2d> polygon;
    polygon.push_back(Vector2d(1, 1));
    polygon.push_back(Vector2d(2, 1));
    polygon.push_back(Vector2d(2, 2));
    polygon.push_back(Vector2d(1, 2));
    Vector2d point(1, 1);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(isPointIn2DPolygon(point, polygon, tol));
}

// checkSegmentPlaneIntersection
// Test per un segmento che interseca il piano
TEST(CheckSegmentPlaneIntersection, SegmentIntersectsPlane) {
    vector<Vector3d> intersections;
    Vector3d planeNormal(0, 0, 1);
    Vector3d planePoint(0, 0, 0);
    Vector3d s1(1, 1, 1);
    Vector3d s2(-1, -1, -1); // Segment intersects the plane at (0,0,0)
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(checkSegmentPlaneIntersection(intersections, planeNormal, planePoint, s1, s2, tol));
    ASSERT_EQ(intersections.size(), 1);
    EXPECT_EQ(intersections[0], Vector3d(0, 0, 0));
}

// Test per un segmento che giace completamente sopra il piano
TEST(CheckSegmentPlaneIntersection, SegmentAbovePlane) {
    vector<Vector3d> intersections;
    Vector3d planeNormal(0, 0, 1);
    Vector3d planePoint(0, 0, 0);
    Vector3d s1(1, 1, 1);
    Vector3d s2(2, 2, 2);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_FALSE(checkSegmentPlaneIntersection(intersections, planeNormal, planePoint, s1, s2, tol));
    ASSERT_EQ(intersections.size(), 0);
}

// Test per un segmento che giace completamente sotto il piano
TEST(CheckSegmentPlaneIntersection, SegmentBelowPlane) {
    vector<Vector3d> intersections;
    Vector3d planeNormal(0, 0, 1);
    Vector3d planePoint(0, 0, 0);
    Vector3d s1(-1, -1, -1);
    Vector3d s2(-2, -2, -2);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_FALSE(checkSegmentPlaneIntersection(intersections, planeNormal, planePoint, s1, s2, tol));
    ASSERT_EQ(intersections.size(), 0);
}

// Test per un segmento che tocca da sotto il piano
TEST(CheckSegmentPlaneIntersection, SegmentTouchingBelowPlane) {
    vector<Vector3d> intersections;
    Vector3d planeNormal(0, 0, 1);
    Vector3d planePoint(0, 0, 0);
    Vector3d s1(0, 0, 0);
    Vector3d s2(-2, -2, -2);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(checkSegmentPlaneIntersection(intersections, planeNormal, planePoint, s1, s2, tol));
    ASSERT_EQ(intersections.size(), 1);
}

// Test per un segmento che tocca da sopra il piano
TEST(CheckSegmentPlaneIntersection, SegmentTouchingAbovePlane) {
    vector<Vector3d> intersections;
    Vector3d planeNormal(0, 0, 1);
    Vector3d planePoint(0, 0, 0);
    Vector3d s1(1, 1, 1);
    Vector3d s2(0, 0, 0);
    double tol=10*numeric_limits<double>::epsilon();
    EXPECT_TRUE(checkSegmentPlaneIntersection(intersections, planeNormal, planePoint, s1, s2, tol));
    ASSERT_EQ(intersections.size(), 1);
}

// findIntersections

// findTraces
// Test per il caso in cui non ci siano intersezioni tra le fratture
TEST(FindTraces, NoIntersections) {
    vector<Trace> traces;
    vector<Fracture> fractures;
    double tol=10*numeric_limits<double>::epsilon();
    unsigned int numTraces = findTraces(traces, fractures, tol);
    EXPECT_EQ(numTraces, 0);
}

// Test per il caso in cui ci siano intersezioni tra le fratture
TEST(FindTraces, IntersectionsPresent) {
    vector<Trace> traces;
    vector<Fracture> fractures;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1, verticesF2;

    // Frattura 1
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F1 = Fracture(1, verticesF1, tol);

    // Frattura 2
    verticesF2.push_back(Vector3d(0.5, 0, 1));
    verticesF2.push_back(Vector3d(0.5, 0, -1));
    verticesF2.push_back(Vector3d(0, 1, 0));
    Fracture F2 = Fracture(2, verticesF2, tol);

    fractures.push_back(F1);
    fractures.push_back(F2);

    unsigned int numTraces = findTraces(traces, fractures, tol);
    EXPECT_EQ(numTraces, 1);
    ASSERT_EQ(traces.size(), 1);
    EXPECT_TRUE(areVectorsEqual(traces[0].extremes[0], Vector3d(0.5, 0, 0), tol));
    EXPECT_TRUE(areVectorsEqual(traces[0].extremes[1], Vector3d(0, 1, 0) , tol));
}

// Test per il caso in cui le fratture si intersecano in un lato
TEST(FindTraces, IntersectionIsOnEdge) {
    vector<Trace> traces;
    vector<Fracture> fractures;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1, verticesF2;

    // Frattura 1
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F1 = Fracture(1, verticesF1, tol);

    // Frattura 2
    verticesF2.push_back(Vector3d(1, 0, 0));
    verticesF2.push_back(Vector3d(1, 1, 0));
    verticesF2.push_back(Vector3d(0, 1, 1));
    verticesF2.push_back(Vector3d(1, 0, 1));
    Fracture F2 = Fracture(2, verticesF2, tol);

    fractures.push_back(F1);
    fractures.push_back(F2);

    unsigned int numTraces = findTraces(traces, fractures, tol);
    EXPECT_EQ(numTraces, 1);
    EXPECT_TRUE(areVectorsEqual(traces[0].extremes[0], Vector3d(1, 0, 0), tol));
    EXPECT_TRUE(areVectorsEqual(traces[0].extremes[1], Vector3d(1, 1, 0), tol));
}

TEST(FindTraces, IntersectionIsOnSinglePoint) {
    vector<Trace> traces;
    vector<Fracture> fractures;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1, verticesF2;

    // Frattura 1
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F1 = Fracture(1, verticesF1, tol);

    // Frattura 2
    verticesF2.push_back(Vector3d(0.5, 0.5, 0));
    verticesF2.push_back(Vector3d(1, 1, 1));
    verticesF2.push_back(Vector3d(2, 1, 1));
    Fracture F2 = Fracture(2, verticesF2, tol);

    fractures.push_back(F1);
    fractures.push_back(F2);

    unsigned int numTraces = findTraces(traces, fractures, tol);
    EXPECT_EQ(numTraces, 0);
}

TEST(FindTraces, IntersectionIsOnVertex) {
    vector<Trace> traces;
    vector<Fracture> fractures;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1, verticesF2;

    // Frattura 1
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F1 = Fracture(1, verticesF1, tol);

    // Frattura 2
    verticesF2.push_back(Vector3d(0, 0, 0));
    verticesF2.push_back(Vector3d(1, 1, 1));
    verticesF2.push_back(Vector3d(2, 1, 1));
    Fracture F2 = Fracture(2, verticesF2, tol);

    fractures.push_back(F1);
    fractures.push_back(F2);

    unsigned int numTraces = findTraces(traces, fractures, tol);
    EXPECT_EQ(numTraces, 0);
}

// sortTraces
// Test per la funzione sortTraces
TEST(SortTraces, WithNotSortedTraces) {
    // Creazione di una frattura di esempio con tracce non ordinate
    Fracture fracture;
    fracture.idFrac = 1;

    // Traccia 1 (non ordinata)
    Trace trace1;
    trace1.idTrace = 1;
    trace1.length = 5;
    fracture.passingTraces.push_back(trace1);

    // Traccia 2 (non ordinata)
    Trace trace2;
    trace2.idTrace = 2;
    trace2.length = 3;
    fracture.passingTraces.push_back(trace2);

    // Traccia 3 (non ordinata)
    Trace trace3;
    trace3.idTrace = 3;
    trace3.length = 7;
    fracture.passingTraces.push_back(trace3);

    // Applica la funzione sortTraces
    vector<Fracture> fractures;
    fractures.push_back(fracture);
    sortTraces(fractures);

    // Verifica che le tracce siano ordinate in modo decrescente per lunghezza
    EXPECT_EQ(fractures[0].passingTraces.size(), 3);
    EXPECT_EQ(fractures[0].passingTraces[0].length, 7);
    EXPECT_EQ(fractures[0].passingTraces[1].length, 5);
    EXPECT_EQ(fractures[0].passingTraces[2].length, 3);
}

// classifyTracePosition
TEST(classifyTracePosition, AbovePlane) {
    Vector3d planePoint(0, 0, 0);
    Vector3d planeNormal(0, 0, 1);
    Vector3d s1(1, 1, 1);
    Vector3d s2(2, 2, 2);
    int result = classifyTracePosition(planePoint, planeNormal, s1, s2);
    EXPECT_EQ(result, 1); // Sopra il piano
}

TEST(classifyTracePosition, BelowPlane) {
    Vector3d planePoint(0, 0, 0);
    Vector3d planeNormal(0, 0, 1);
    Vector3d s1(-1, -1, -1);
    Vector3d s2(-2, -2, -2);
    int result = classifyTracePosition(planePoint, planeNormal, s1, s2);
    EXPECT_EQ(result, -1); // Sotto il piano
}

TEST(classifyTracePosition, CrossingPlane) {
    Vector3d planePoint(0, 0, 0);
    Vector3d planeNormal(0, 0, 1);
    Vector3d s1(-1, -1, 1);
    Vector3d s2(1, 1, -1);
    int result = classifyTracePosition(planePoint, planeNormal, s1, s2);
    EXPECT_EQ(result, 0); // Attraversa il piano
}

// checkTraceTips
TEST(checkTraceTips, NonPassingTrace) {
    vector<Trace> traces;
    vector<Fracture> fractures;
    vector<Vector3d> verticesF;
    double tol=10*numeric_limits<double>::epsilon();

    verticesF.push_back(Vector3d(0, 0, 0));
    verticesF.push_back(Vector3d(1, 0, 0));
    verticesF.push_back(Vector3d(1, 1, 0));
    verticesF.push_back(Vector3d(0, 1, 0));
    Fracture F = Fracture(1, verticesF, tol);

    Trace T;
    T.idTrace = 1;
    T.extremes.push_back(Vector3d(0.5, 0.5, 0)); // Punto interno alla frattura
    T.extremes.push_back(Vector3d(0.5, 1, 0)); // Punto sul bordo della frattura
    bool result = checkTraceTips(F, T, tol);
    EXPECT_TRUE(result); // La traccia non è passante
}

TEST(checkTraceTips, PassingTrace) {
    Fracture F;
    F.idFrac = 1;
    F.vertices.push_back(Vector3d(0, 0, 0));
    F.vertices.push_back(Vector3d(1, 0, 0));
    F.vertices.push_back(Vector3d(1, 1, 0));
    F.vertices.push_back(Vector3d(0, 1, 0));

    Trace T;
    T.idTrace = 1;
    T.extremes.push_back(Vector3d(0.5, 0, 0)); // Punto all'interno della frattura
    T.extremes.push_back(Vector3d(0.5, 1, 0)); // Punto all'interno della frattura

    double tol=10*numeric_limits<double>::epsilon();
    bool result = checkTraceTips(F, T, tol);
    EXPECT_FALSE(result); // La traccia è passante
}

TEST(checkTraceTips, TraceCompletelyNotPassing) {
    Fracture F;
    F.idFrac = 1;
    F.vertices.push_back(Vector3d(0, 0, 0));
    F.vertices.push_back(Vector3d(1, 0, 0));
    F.vertices.push_back(Vector3d(1, 1, 0));
    F.vertices.push_back(Vector3d(0, 1, 0));

    Trace T;
    T.idTrace = 1;
    T.extremes.push_back(Vector3d(0.5, 0.5, 0));
    T.extremes.push_back(Vector3d(0.5, 0.3, 0));

    double tol=10*numeric_limits<double>::epsilon();
    bool result = checkTraceTips(F, T, tol);
    EXPECT_TRUE(result);
}

// Test per traccia sul bordo di una frattura
TEST(checkTraceTips, TraceIsEdge) {
    Fracture F;
    F.idFrac = 1;
    F.vertices.push_back(Vector3d(0, 0, 0));
    F.vertices.push_back(Vector3d(1, 0, 0));
    F.vertices.push_back(Vector3d(1, 1, 0));
    F.vertices.push_back(Vector3d(0, 1, 0));

    Trace T;
    T.idTrace = 1;
    T.extremes.push_back(Vector3d(0, 0, 0));
    T.extremes.push_back(Vector3d(1, 0, 0));

    double tol=10*numeric_limits<double>::epsilon();
    bool result = checkTraceTips(F, T, tol);
    EXPECT_FALSE(result);
}

// addTraceToFractures
TEST(AddTraceToFractures, PassingTrace) {
    double tol=10*numeric_limits<double>::epsilon();
    vector<Vector3d> verticesF1, verticesF2;

    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F1 = Fracture(1, verticesF1, tol);

    verticesF2.push_back(Vector3d(0.5, 0, 0));
    verticesF2.push_back(Vector3d(0.5, 1, 0));
    verticesF2.push_back(Vector3d(0, 1, 1));
    verticesF2.push_back(Vector3d(0, 0, 1));
    Fracture F2 = Fracture(2, verticesF2, tol);

    Trace T;
    T.idTrace = 1;
    T.extremes.push_back(Vector3d(0.5, 0, 0)); // Punto all'interno della frattura
    T.extremes.push_back(Vector3d(0.5, 1, 0)); // Punto all'interno della frattura

    addTraceToFractures(F1, F2, T, tol);

    // Assicurati che la traccia sia stata aggiunta alle tracce non passanti della frattura F1
    ASSERT_EQ(F1.passingTraces.size(), 1);
    EXPECT_EQ(F1.passingTraces[0].idTrace, 1);
}

TEST(AddTraceToFractures, NotPassingTrace) {
    double tol=10*numeric_limits<double>::epsilon();
    vector<Vector3d> verticesF1, verticesF2;

    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F1 = Fracture(1, verticesF1, tol);

    verticesF2.push_back(Vector3d(0.5, 0.5, 0));
    verticesF2.push_back(Vector3d(0.5, 0.3, 0));
    verticesF2.push_back(Vector3d(0, 1, 1));
    verticesF2.push_back(Vector3d(0, 0, 1));
    Fracture F2 = Fracture(2, verticesF2, tol);

    Trace T;
    T.idTrace = 1;
    T.extremes.push_back(Vector3d(0.5, 0.5, 0));
    T.extremes.push_back(Vector3d(0.5, 0.3, 0));
    T.length = 0.2;


    addTraceToFractures(F1, F2, T, tol);

    // Assicurati che la traccia sia stata aggiunta alle tracce passanti della frattura F1
    ASSERT_EQ(F1.notPassingTraces.size(), 1);
    EXPECT_EQ(F1.notPassingTraces[0].idTrace, 1);
}

// findLineSegmentIntersection
TEST(FindLineSegmentIntersection, IntersectionInsideSegment) {
    Vector3d intersection;
    Vector3d planeNormal(0, 0, 1);
    Vector3d t1(0, 0, 0);
    Vector3d t2(0, 1, 0);
    Vector3d s1(0, 0.5, 0); // Punto interno al segmento
    Vector3d s2(0, 1, 0);
    double tol=10*numeric_limits<double>::epsilon();

    // bool result = findLineSegmentIntersection(intersection, planeNormal, t1, t2, s1, s2, tol);

    // EXPECT_TRUE(result);
    EXPECT_EQ(intersection, s1);
}

TEST(FindLineSegmentIntersection, intersectionNotPresent) {
    Vector3d intersection;
    Vector3d planeNormal(0, 0, 1);
    Vector3d t1(0, 0, 0);
    Vector3d t2(0, 1, 0);
    Vector3d s1(1, 1, 0); // Punto esterno al segmento
    Vector3d s2(1, 2, 0);
    double tol=10*numeric_limits<double>::epsilon();

    // bool result = findLineSegmentIntersection(intersection, planeNormal, t1, t2, s1, s2, tol);

    // EXPECT_FALSE(result);
}

TEST(FindLineSegmentIntersection, IntersectionOnSegmentExtreme) {
    Vector3d intersection;
    Vector3d planeNormal(1, 0, 0);
    Vector3d t1(0, 0, 0);
    Vector3d t2(0, 1, 0);
    Vector3d s1(0, 0, 0); // Punto coincidente con un'estremità del segmento
    Vector3d s2(0, 0.5, 0);
    double tol=10*numeric_limits<double>::epsilon();

    // bool result = findLineSegmentIntersection(intersection, planeNormal, t1, t2, s1, s2, tol);

    // EXPECT_TRUE(result);
    EXPECT_EQ(intersection, s1);
}

// splitFracture
TEST(SplitFracture, SplitFractureByPassing) {
    vector<Fracture> subFractures;
    vector<Vector3d> cutPoints;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1;
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F1 = Fracture(1, verticesF1, tol);

    Vector3d t1(0.5, 0, 0);
    Vector3d t2(0.5, 1, 0);

    splitFracture(subFractures, cutPoints, F1, t1, t2, tol);

    ASSERT_EQ(subFractures.size(), 2); // Dovrebbero esserci due sotto-fratture

    // Verifica che le sotto-fratture siano correttamente create
    EXPECT_EQ(subFractures[0].idFrac, 11); // Id della prima sotto-frattura
    EXPECT_EQ(subFractures[1].idFrac, 12); // Id della seconda sotto-frattura

    EXPECT_EQ(subFractures[0].vertices[0], Vector3d(0, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[1], Vector3d(0.5, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[2], Vector3d(0.5, 1, 0));
    EXPECT_EQ(subFractures[0].vertices[3], Vector3d(0, 1, 0));

    EXPECT_EQ(subFractures[1].vertices[0], Vector3d(0.5, 0, 0));
    EXPECT_EQ(subFractures[1].vertices[1], Vector3d(1, 0, 0));
    EXPECT_EQ(subFractures[1].vertices[2], Vector3d(1, 1, 0));
    EXPECT_EQ(subFractures[1].vertices[3], Vector3d(0.5, 1, 0));

    // Verifica che i punti di taglio siano correttamente identificati
    ASSERT_EQ(cutPoints.size(), 2);
    EXPECT_EQ(cutPoints[0], Vector3d(0.5, 0, 0));
    EXPECT_EQ(cutPoints[1], Vector3d(0.5, 1, 0));
}

// Split nel caso una traccia sia passante ma con il taglio con un punto interno e uno esterno alla frattura
TEST(SplitFracture, SplitFractureByPassingOuter) {
    vector<Fracture> subFractures;
    vector<Vector3d> cutPoints;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1;
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F = Fracture(1, verticesF1, tol);

    Vector3d t1(0.5, -0.5, 0);
    Vector3d t2(0.5, 1, 0);

    splitFracture(subFractures, cutPoints, F, t1, t2, tol);

    ASSERT_EQ(subFractures.size(), 2); // Dovrebbero esserci due sotto-fratture

    // Verifica che le sotto-fratture siano correttamente create
    EXPECT_EQ(subFractures[0].idFrac, 11); // Id della prima sotto-frattura
    EXPECT_EQ(subFractures[1].idFrac, 12); // Id della seconda sotto-frattura

    EXPECT_EQ(subFractures[0].vertices[0], Vector3d(0, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[1], Vector3d(0.5, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[2], Vector3d(0.5, 1, 0));
    EXPECT_EQ(subFractures[0].vertices[3], Vector3d(0, 1, 0));

    EXPECT_EQ(subFractures[1].vertices[0], Vector3d(0.5, 0, 0));
    EXPECT_EQ(subFractures[1].vertices[1], Vector3d(1, 0, 0));
    EXPECT_EQ(subFractures[1].vertices[2], Vector3d(1, 1, 0));
    EXPECT_EQ(subFractures[1].vertices[3], Vector3d(0.5, 1, 0));

    // Verifica che i punti di taglio siano correttamente identificati
    ASSERT_EQ(cutPoints.size(), 2);
    EXPECT_EQ(cutPoints[0], Vector3d(0.5, 0, 0));
    EXPECT_EQ(cutPoints[1], Vector3d(0.5, 1, 0));
}

// Split nel caso una traccia sia non passante ma con il taglio con entrambi i punti interni alla frattura
TEST(SplitFracture, SplitFractureByNotPassingInner) {
    vector<Fracture> subFractures;
    vector<Vector3d> cutPoints;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1;
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F = Fracture(1, verticesF1, tol);

    Vector3d t1(0.5, 0.5, 0);
    Vector3d t2(0.5, 0.3, 0);

    splitFracture(subFractures, cutPoints, F, t1, t2, tol);

    ASSERT_EQ(subFractures.size(), 2); // Dovrebbero esserci due sotto-fratture

    // Verifica che le sotto-fratture siano correttamente create
    EXPECT_EQ(subFractures[0].idFrac, 11); // Id della prima sotto-frattura
    EXPECT_EQ(subFractures[1].idFrac, 12); // Id della seconda sotto-frattura

    EXPECT_EQ(subFractures[0].vertices[0], Vector3d(0, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[1], Vector3d(0.5, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[2], Vector3d(0.5, 1, 0));
    EXPECT_EQ(subFractures[0].vertices[3], Vector3d(0, 1, 0));

    EXPECT_EQ(subFractures[1].vertices[0], Vector3d(0.5, 0, 0));
    EXPECT_EQ(subFractures[1].vertices[1], Vector3d(1, 0, 0));
    EXPECT_EQ(subFractures[1].vertices[2], Vector3d(1, 1, 0));
    EXPECT_EQ(subFractures[1].vertices[3], Vector3d(0.5, 1, 0));

    // Verifica che i punti di taglio siano correttamente identificati
    ASSERT_EQ(cutPoints.size(), 2);
    EXPECT_EQ(cutPoints[0], Vector3d(0.5, 0, 0));
    EXPECT_EQ(cutPoints[1], Vector3d(0.5, 1, 0));
}

// Split nel caso una traccia sia non passante ma con il taglio con un punto interno e uno esterno alla frattura
TEST(SplitFracture, SplitFractureByNotPassingOuter) {
    vector<Fracture> subFractures;
    vector<Vector3d> cutPoints;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1;
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F = Fracture(1, verticesF1, tol);

    Vector3d t1(0.5, 0.5, 0);
    Vector3d t2(0.5, -0.5, 0);

    splitFracture(subFractures, cutPoints, F, t1, t2, tol);

    ASSERT_EQ(subFractures.size(), 2); // Dovrebbero esserci due sotto-fratture

    // Verifica che le sotto-fratture siano correttamente create
    EXPECT_EQ(subFractures[0].idFrac, 11); // Id della prima sotto-frattura
    EXPECT_EQ(subFractures[1].idFrac, 12); // Id della seconda sotto-frattura

    EXPECT_EQ(subFractures[0].vertices[0], Vector3d(0, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[1], Vector3d(0.5, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[2], Vector3d(0.5, 1, 0));
    EXPECT_EQ(subFractures[0].vertices[3], Vector3d(0, 1, 0));

    EXPECT_EQ(subFractures[1].vertices[0], Vector3d(0.5, 0, 0));
    EXPECT_EQ(subFractures[1].vertices[1], Vector3d(1, 0, 0));
    EXPECT_EQ(subFractures[1].vertices[2], Vector3d(1, 1, 0));
    EXPECT_EQ(subFractures[1].vertices[3], Vector3d(0.5, 1, 0));

    // Verifica che i punti di taglio siano correttamente identificati
    ASSERT_EQ(cutPoints.size(), 2);
    EXPECT_EQ(cutPoints[0], Vector3d(0.5, 0, 0));
    EXPECT_EQ(cutPoints[1], Vector3d(0.5, 1, 0));
}

// Test per la funzione splitFracture con nessuna intersezione tra il taglio e i lati della frattura
TEST(SplitFracture, NoIntersectionParallel) {
    vector<Fracture> subFractures;
    vector<Vector3d> cutPoints;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1;
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F = Fracture(1, verticesF1, tol);

    Vector3d t1(2, 2, 0); // Estremi del taglio al di fuori della frattura
    Vector3d t2(2, 3, 0);


    splitFracture(subFractures, cutPoints, F, t1, t2, tol);

    EXPECT_EQ(subFractures.size(), 1); // Dovrebbe esserci una sola sotto-frattura
    EXPECT_EQ(subFractures[0].idFrac, 11); // L'id della sotto-frattura dovrebbe essere lo stesso dell'originale

    EXPECT_EQ(subFractures[0].vertices[0], Vector3d(0, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[1], Vector3d(1, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[2], Vector3d(1, 1, 0));
    EXPECT_EQ(subFractures[0].vertices[3], Vector3d(0, 1, 0));

    EXPECT_EQ(cutPoints.size(), 0); // Non ci dovrebbero essere punti di taglio
}

// Test per la funzione splitFracture con nessuna intersezione tra il taglio e i lati della frattura
TEST(SplitFracture, NoIntersection) {
    vector<Fracture> subFractures;
    vector<Vector3d> cutPoints;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1;
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F = Fracture(1, verticesF1, tol);

    Vector3d t1(2, 2, 0); // Estremi del taglio al di fuori della frattura
    Vector3d t2(3, 3, 0);


    splitFracture(subFractures, cutPoints, F, t1, t2, tol);

    EXPECT_EQ(subFractures.size(), 1); // Dovrebbe esserci una sola sotto-frattura
    EXPECT_EQ(subFractures[0].idFrac, 11); // L'id della sotto-frattura dovrebbe essere lo stesso dell'originale

    EXPECT_EQ(subFractures[0].vertices[0], Vector3d(1, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[1], Vector3d(0, 0, 0));
    EXPECT_EQ(subFractures[0].vertices[2], Vector3d(0, 1, 0));
    EXPECT_EQ(subFractures[0].vertices[3], Vector3d(1, 1, 0));

    EXPECT_EQ(cutPoints.size(), 0); // Non ci dovrebbero essere punti di taglio
}

TEST(CuttingFracture, test1) {
    vector<Fracture> resultFractures;
    deque<Trace> traces;
    double tol=10*numeric_limits<double>::epsilon();

    vector<Vector3d> verticesF1;
    verticesF1.push_back(Vector3d(0, 0, 0));
    verticesF1.push_back(Vector3d(1, 0, 0));
    verticesF1.push_back(Vector3d(1, 1, 0));
    verticesF1.push_back(Vector3d(0, 1, 0));
    Fracture F = Fracture(1, verticesF1, tol);

    Trace T1 =  Trace(1, {{0, 0, 0}, {1, 1, 0}});
    Trace T2 =  Trace(1, {{0.5, 0.5, 0}, {0.5, 0, 0}});
    Trace T3 =  Trace(1, {{0.8, 0, 0}, {0.8, 1, 0}});
    traces.push_back(T1);
    traces.push_back(T2);
    traces.push_back(T3);

    sort(traces.begin(), traces.end(), [] (const Trace& a, const Trace& b) { return a.length > b.length; }); //funzione inline dal [] in poi. non chiama lo stack frame. è la funzione di confronto

    cuttingFracture(resultFractures, F, traces, tol);

    ASSERT_EQ(resultFractures.size(), 3);

    EXPECT_EQ(resultFractures[0].vertices[0], Vector3d(0.8, 0, 0));
    EXPECT_EQ(resultFractures[0].vertices[1], Vector3d(1, 0, 0));
    EXPECT_EQ(resultFractures[0].vertices[2], Vector3d(1, 1, 0));
    EXPECT_EQ(resultFractures[0].vertices[3], Vector3d(0.8, 1, 0));


}


TEST(PolygonalMeshTest, PointInsidePolygon) {
    PolygonalMesh mesh;
    double tol=10*numeric_limits<double>::epsilon();

    // Define vertices
    Vector3d v1(0, 0, 0);
    Vector3d v2(1, 0, 0);
    Vector3d v3(0, 1, 0);

    // Add Cell0D vertices
    Cell0D cell0D1 = mesh.addCell0D(v1);
    Cell0D cell0D2 = mesh.addCell0D(v2);
    Cell0D cell0D3 = mesh.addCell0D(v3);

    // Add Cell1D edges
    Cell1D cell1D1 = mesh.addCell1D(cell0D1, cell0D2);
    Cell1D cell1D2 = mesh.addCell1D(cell0D2, cell0D3);
    Cell1D cell1D3 = mesh.addCell1D(cell0D3, cell0D1);

    // Define a plane for the Cell2D
    Vector3d normal = Vector3d(0, 0, 1);

    // Add Cell2D polygon
    vector<Cell0D> vertices = {cell0D1, cell0D2, cell0D3};
    vector<Cell1D> edges = {cell1D1, cell1D2, cell1D3};
    Cell2D cell2D = mesh.addCell2D(normal, vertices, edges);

    // Test a point inside the polygon
    Vector3d pointInside = Vector3d(0.2, 0.2, 0);

    Cell2D foundCell;
    bool found = findCellContainingPoint(foundCell, mesh, pointInside, tol);
    ASSERT_TRUE(found);
    EXPECT_EQ(foundCell.id, cell2D.id);

    // Test a point outside the polygon
    Vector3d pointOutside(2, 2, 0);
    found = findCellContainingPoint(foundCell, mesh, pointOutside, tol);
    EXPECT_FALSE(found);
}

TEST(PolygonalMeshTest, PointInsidePolygon2) {
    PolygonalMesh mesh;
    double tol=10*numeric_limits<double>::epsilon()*10000;

    // Define vertices
    Vector3d v1(0, 0, 1);
    Vector3d v2(0, 1, 1);
    Vector3d v3(0, 1, 0);

    // Add Cell0D vertices
    Cell0D cell0D1 = mesh.addCell0D(v1);
    Cell0D cell0D2 = mesh.addCell0D(v2);
    Cell0D cell0D3 = mesh.addCell0D(v3);

    // Add Cell1D edges
    Cell1D cell1D1 = mesh.addCell1D(cell0D1, cell0D2);
    Cell1D cell1D2 = mesh.addCell1D(cell0D2, cell0D3);
    Cell1D cell1D3 = mesh.addCell1D(cell0D3, cell0D1);

    // Define a plane for the Cell2D
    Vector3d normal = Vector3d(0, 0, 1);

    // Add Cell2D polygon
    vector<Cell0D> vertices = {cell0D1, cell0D2, cell0D3};
    vector<Cell1D> edges = {cell1D1, cell1D2, cell1D3};
    Cell2D cell2D = mesh.addCell2D(normal, vertices, edges);

    // Test a point inside the polygon
    Vector3d pointInside = Vector3d(0.27578887488160, 0.44773812760435, 0.72421112511839);

    Cell2D foundCell;
    bool found = findCellContainingPoint(foundCell, mesh, pointInside, tol);
    ASSERT_TRUE(found);
    EXPECT_EQ(foundCell.id, cell2D.id);
}
// printTraces

// printFractures

// printTracesToFile

// printFracturesToFile

// transformChildrenFracturesToMesh

// saveMesh
