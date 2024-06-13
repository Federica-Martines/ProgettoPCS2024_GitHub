#pragma once

#include "PolygonalMesh.hpp"
#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace PolygonalLibrary;

namespace GeometryLibrary {

// given 3 points not aligned returns the normal to the plane passing in the 3 points
Vector3d findNormal(const Vector3d p1, const Vector3d p2, const Vector3d p3);

// returns the plane the fracture is lying on (done to prevent checking all the time and for debugging
unsigned int findLyingPlane(const Vector3d n, double tol);

//scrivi sempre "struttura" (PUBBLICA), non "classe" (privata)
struct BoundingSphere {
    Vector3d centroid;
    double radius;
    BoundingSphere(Vector3d centroid, double radius){
        this->centroid = centroid;
        this->radius = radius;
    }
};

struct Trace{
    unsigned int idTrace;
    unsigned int idGenerator1;
    unsigned int idGenerator2;
    double length;
    bool tips;
    vector <Vector3d> extremes={}; //coordinate dei vertici della frattura

    Trace()=default;

    // debug only
    Trace(unsigned int idTrace, vector <Vector3d> extremes) {
        this->idTrace = idTrace;
        this->extremes = extremes;
        this->length = (extremes[0] - extremes[1]).norm();
    }

    Trace(unsigned int idTrace, unsigned int idGenerator1, unsigned int idGenerator2, vector <Vector3d> extremes) {
        this->idTrace = idTrace;
        this->idGenerator1 = idGenerator1;
        this->idGenerator2 = idGenerator2;
        this->extremes = extremes;
        this->length = (extremes[0] - extremes[1]).norm();
    }
};

//struttura
struct Fracture{
    unsigned int idFrac;
    unsigned int numVertices;
    Vector3d normal;
    unsigned int lyingPlane; //piano su cui si appoggia
    vector<Vector3d> vertices={}; //coordinate dei vertici della frattura
    vector<Trace> passingTraces={}; //vettore con gli id delle tracce passanti per la frattura corrente
    vector<Trace> notPassingTraces={};

    //costruttore (o questa: calcoli la roba)
    Fracture()=default;
    Fracture(unsigned int id, vector<Vector3d> vertices, double tol) {
        this->idFrac = id;
        this->numVertices = vertices.size();
        this->vertices = vertices;
        this->normal = findNormal(vertices[0], vertices[1], vertices[2]);
        this->lyingPlane = findLyingPlane(this->normal, tol);
    }

    //(o questa: hai già le cose. quando fai i tagli chiami il costruttore della frattura)
    Fracture(unsigned int id, vector<Vector3d> vertices, Vector3d normal, unsigned int lyingPlane) {
        this->idFrac = id;
        this->numVertices = vertices.size();
        this->vertices = vertices;
        this->normal = normal;
        this->lyingPlane = lyingPlane;
    }

    bool checkFractureEdges(double tol);
};

PolygonalMesh convertFractureToMesh(const Fracture& fracture, double tol);

BoundingSphere computeBoundingSphere(const std::vector<Vector3d>& vertices);

bool spheresIntersect(const BoundingSphere& sphere1, const BoundingSphere& sphere2);

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

bool existDirectionSegmentIntersection(Vector3d t1, Vector3d t2, Vector3d s1, Vector3d s2, double tol) ;

int findLineSegmentIntersection(Vector3d& intersection,
                                double& alpha,
                                double& beta,
                                const Trace cut,
                                const Cell1D edge,
                                double tol
                                );

bool areVectorsEqual(Vector3d v1, Vector3d v2, double tol);
}

bool pointInPolygon(const Vector3d& point, const Cell2D& cell);
