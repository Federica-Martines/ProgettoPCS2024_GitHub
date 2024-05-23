#pragma once

#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;

namespace Geometry {

struct BoundingSphere {
    Vector3d center;
    double radius;
};

struct Trace{
    unsigned int idTrace;
    unsigned int idGenerator1;
    unsigned int idGenerator2;
    double length;
    bool tips;
    vector <Vector3d> extremes={}; //coordinate dei vertici della frattura

    Trace()=default;
};

struct Fracture{
    unsigned int idFrac;
    unsigned int numVertices;
    Vector3d normal;
    unsigned int lyingPlane;
    vector <Vector3d> vertices={}; //coordinate dei vertici della frattura
    vector <Trace> passingTraces={}; //vettore con gli id delle tracce passanti per la frattura corrente
    vector <Trace> notPassingTraces={};

    Fracture()=default;

    bool checkFractureEdges(double tol);
};

BoundingSphere computeBoundingSphere(const std::vector<Vector3d>& vertices);

bool spheresIntersect(const BoundingSphere& sphere1, const BoundingSphere& sphere2);


// given 3 points not aligned returns the normal to the plane passing in the 3 points
Vector3d findNormal(const Vector3d p1, const Vector3d p2, const Vector3d p3);

// returns the plane the fracture is lying on (done to prevent checking all the time and for debugging
unsigned int findLyingPlane(const Vector3d n, double tol);

// Function to project a 3D point onto the XY plane
Vector2d projectOntoXY(const Vector3d& point);

// Function to project a 3D point onto the XZ plane
Vector2d projectOntoXZ(const Vector3d& point);

// Function to project a 3D point onto the YZ plane
Vector2d projectOntoYZ(const Vector3d& point);

bool isPointOn2DSegment(const Vector2d& point, const Vector2d& s1, const Vector2d& s2, double tol);

bool isPointOn3DSegment(const Vector3d& point, const Vector3d& s1, const Vector3d& s2, double tol);

void projectIntersection(Vector2d& projIntersection, Fracture F, Vector3d intersection);

void projectVertices(vector<Vector2d>& projVertices, Fracture F);

bool isPointIn2DPolygon(const Vector2d& point, const vector<Vector2d>& polygon, double tol);

int classifyTracePosition(const Vector3d& planePoint, const Vector3d& planeNormal, const Vector3d& s1, const Vector3d& s2);

bool findLineSegmentIntersection(Vector3d& intersection, const Vector3d& l1, const Vector3d& l2, const Vector3d& s1, const Vector3d& s2, double tol);


}
