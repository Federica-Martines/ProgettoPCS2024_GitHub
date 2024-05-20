#include "GeometryLibrary.hpp"
#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
using namespace Geometry;

namespace Geometry {

bool Fracture::checkFractureEdges(double tol){

    //controlliamo che non ci siano lati di lunghezza nulla.
    for (unsigned int k=0; k<this->numVertices-1; k++){

        //differenza tra due vertici consecutivi
        Vector3d edge = (this->vertices)[k]-(this->vertices)[k+1];
        if (edge[0]*edge[0]+edge[1]*edge[1]+edge[2]*edge[2]<tol*tol){
            cerr << "la frattura " << this->idFrac << " ha lati di lunghezza nulla" << endl;
            //costruttore particolare per differenziare le fratture con problemi
            return false;
        }
    }
    Vector3d edgeF = (this->vertices)[0]-(this->vertices)[this->numVertices-1]; //faccio lo stesso per il primo e l'ultimo vertice
    if (edgeF[0]*edgeF[0]+edgeF[1]*edgeF[1]+edgeF[2]*edgeF[2]<tol*tol){
        cerr << "la frattura " << this->idFrac << " ha lati di lunghezza nulla" << endl;
        return false;
    }
    return true;
};

// given 3 points not aligned returns the normal to the plane passing in the 3 points
Vector3d findNormal(const Vector3d p1, const Vector3d p2, const Vector3d p3) {

    // Calcolo i vettori che generano la normale
    Vector3d u1 = p3 - p1;
    Vector3d v1 = p2 - p1;
    return u1.cross(v1).normalized();
}

unsigned int findLyingPlane(const Vector3d n, double tol) {

    Vector3d eX = {1,0,0};
    Vector3d eY = {0,1,0};
    Vector3d eZ = {0,0,1};

    if (abs(n.dot(eX)) < tol && abs(n.dot(eY)) < tol) {
        return 1;
    }
    if (abs(n.dot(eX)) < tol && abs(n.dot(eZ)) < tol) {
        return 2;
    }
    if (abs(n.dot(eY)) < tol && abs(n.dot(eZ)) < tol) {
        return 3;
    }

    return 0;
}

// Function to project a 3D point onto the XY plane
Vector2d projectOntoXY(const Vector3d& point)
{
    return Vector2d(point.x(), point.y());
}

// Function to project a 3D point onto the XZ plane
Vector2d projectOntoXZ(const Vector3d& point)
{
    return Vector2d(point.x(), point.z());
}
// Function to project a 3D point onto the YZ plane
Vector2d projectOntoYZ(const Vector3d& point)
{
    return Vector2d(point.y(), point.z());
}


void projectIntersection(Vector2d& projIntersection, Fracture F, Vector3d intersection) {

    switch (F.lyingPlane) {
    case 0:
    case 1:
        projIntersection = projectOntoXY(intersection);
        break;
    case 2:
        projIntersection = projectOntoXZ(intersection);
        break;
    case 3:
        projIntersection = projectOntoYZ(intersection);
        break;
    }

    return;

}

void projectVertices(vector<Vector2d>& projVertices, Fracture F) {

    switch (F.lyingPlane) {
    case 0:
    case 1:
        for (unsigned int ver = 0; ver < F.vertices.size(); ver++) {
            projVertices.push_back(projectOntoXY(F.vertices[ver]));
        }
        break;
    case 2:
        for (unsigned int ver = 0; ver < F.vertices.size(); ver++) {
            projVertices.push_back(projectOntoXZ(F.vertices[ver]));
        }
        break;
    case 3:
        for (unsigned int ver = 0; ver < F.vertices.size(); ver++) {
            projVertices.push_back(projectOntoYZ(F.vertices[ver]));
        }
        break;
    }

    return;
}


bool isPointOn2DSegment(const Vector2d& point, const Vector2d& s1, const Vector2d& s2, double tol) {
    Vector2d v1 = point - s1;
    Vector2d v2 = s2 - s1;
    double crossProduct = v1.x() * v2.y() - v1.y() * v2.x();
    if (fabs(crossProduct) > tol) return false; // Point is not collinear with the segment

    double dotProduct = v1.dot(v2);
    if (dotProduct < -tol || dotProduct > v2.squaredNorm() + tol) return false; // Point is not within the segment bounds

    return true;
}

bool isPointOn3DSegment(const Vector3d& point, const Vector3d& s1, const Vector3d& s2, double tol) {
    Vector3d v1 = point - s1;
    Vector3d v2 = s2 - s1;

    // Calculate cross product
    Vector3d crossProduct = v1.cross(v2);
    double crossProductNorm = crossProduct.norm();

    // Check if the cross product magnitude is greater than tolerance
    if (crossProductNorm > tol) return false; // Point is not collinear with the segment

    double dotProduct = v1.dot(v2);
    if (dotProduct < -tol || dotProduct > v2.squaredNorm() + tol) return false; // Point is not within the segment bounds

    return true;
}

bool isPointIn2DPolygon(const Vector2d& point, const vector<Vector2d>& polygon, double tol) {
    int crossings = 0;
    int n = polygon.size();
    for (int i = 0; i < n; i++) {
        const Vector2d& p1 = polygon[i];
        const Vector2d& p2 = polygon[(i + 1) % n];

        if (isPointOn2DSegment(point, p1, p2, tol))
            return true; // Point lies on an edge

        if (fabs(p1.y() - p2.y()) < tol) // If the segment is horizontal, skip
            continue;

        if (point.y() < min(p1.y(), p2.y()) - tol) // If point is below the segment, skip
            continue;

        if (point.y() >= max(p1.y(), p2.y()) + tol) // If point is above the segment, skip
            continue;

        double x_intersect = (point.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();
        if (x_intersect > point.x() - tol) {
            if (p1.y() > p2.y()) crossings++;
            else crossings--;
        }
    }
    return crossings != 0;
}


}
