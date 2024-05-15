#pragma once

#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;

namespace Geometry {
struct Fracture{
    unsigned int idFrac;
    unsigned int numVertices;
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

// Function to project a 3D point onto the XY plane
Vector2d projectOntoXY(const Vector3d& point);

// Function to project a 3D point onto the XZ plane
Vector2d projectOntoXZ(const Vector3d& point);

// Function to project a 3D point onto the YZ plane
Vector2d projectOntoYZ(const Vector3d& point);

void projectIntVer(Vector2d& projIntersections, vector<Vector2d>& projVertices, Fracture F1, Vector3d n1, Vector3d intersection, double tol);

bool isPointIn2DPolygon(const Vector2d& point, const vector<Vector2d>& polygon);


}

