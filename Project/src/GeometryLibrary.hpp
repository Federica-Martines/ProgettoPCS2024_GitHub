#pragma once

#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;

namespace Geometry {
struct Fracture{
    unsigned int idFrac;
    unsigned int numVertices;
    Vector3d normal;
    unsigned int lyingPlane;
    vector <Vector3d> vertices={}; //coordinate dei vertici della frattura
    vector <unsigned int> passingTraces={}; //vettore con gli id delle tracce passanti per la frattura corrente
    vector <unsigned int> notPassingTraces={};

    Fracture()=default;

    bool checkFractureEdges(double tol);
};

struct Trace{
    unsigned int idTrace;
    unsigned int idGenerator1;
    unsigned int idGenerator2;
    vector <Vector3d> extremes={}; //coordinate dei vertici della frattura

    Trace()=default;
};

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


void projectIntersection(Vector2d& projIntersection, Fracture F, Vector3d intersection);

void projectVertices(vector<Vector2d>& projVertices, Fracture F);

bool isPointIn2DPolygon(const Vector2d& point, const vector<Vector2d>& polygon, double tol);


}

