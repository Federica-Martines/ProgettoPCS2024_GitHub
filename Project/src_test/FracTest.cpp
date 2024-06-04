#include <gtest/gtest.h>
#include "../src/GeometryLibrary.hpp" // Include the header where Fracture and readFractures are defined
#include "../src/input-output.hpp"

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


