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


bool isPointIn2DPolygon(const Vector2d& point, const vector<Vector2d>& polygon)
{
    bool isInside = false; // Initialize the flag to false

    // Loop through each edge of the polygon
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
    {
        // Get the current vertex and the previous vertex
        Vector2d currentVertex = polygon[i];
        Vector2d previousVertex = polygon[j];

        // Check if the point is within the edge's y-bounds
        bool isWithinYBounds = (currentVertex.y() > point.y()) != (previousVertex.y() > point.y());

        // Calculate the x-coordinate of the point where the horizontal line through the point intersects the edge
        double intersectionX = (previousVertex.x() - currentVertex.x()) * (point.y() - currentVertex.y()) / (previousVertex.y() - currentVertex.y()) + currentVertex.x();

        // If the point is within the y-bounds and to the left of the intersection, flip the flag
        if (isWithinYBounds && point.x() < intersectionX)
        {
            isInside = !isInside;
        }
    }

    // Return the final result
    return isInside;
}

void projectIntVer(Vector2d& projIntersection, vector<Vector2d>& projVertices, Fracture F1, Vector3d n1, Vector3d intersection, double tol) {

    Vector3d eX = {1,0,0};
    Vector3d eY = {0,1,0};
    Vector3d eZ = {0,0,1};

    // controlliamo se il piano contenente F1 è ortogonale al piano XY
    if (abs(n1.dot(eX)) < tol && abs(n1.dot(eY)) < tol) {

        projIntersection = projectOntoXY(intersection);
        for (unsigned int ver = 0; ver < F1.vertices.size(); ver++) {
            projVertices.push_back(projectOntoXY(F1.vertices[ver]));
        }
        return;
    }
    if (abs(n1.dot(eX)) < tol && abs(n1.dot(eZ)) < tol) {

        projIntersection = projectOntoXZ(intersection);
        for (unsigned int ver = 0; ver < F1.vertices.size(); ver++) {
            projVertices.push_back(projectOntoXZ(F1.vertices[ver]));
        }
        return;
    }
    if (abs(n1.dot(eY)) < tol && abs(n1.dot(eZ)) < tol) {

        projIntersection =projectOntoYZ(intersection);
        for (unsigned int ver = 0; ver < F1.vertices.size(); ver++) {
            projVertices.push_back(projectOntoYZ(F1.vertices[ver]));
        }
        return;
    }

    // se non è ortogonale a nessun piano ne scegliamo uno
   projIntersection = projectOntoXY(intersection);
    for (unsigned int ver = 0; ver < F1.vertices.size(); ver++) {
        projVertices.push_back(projectOntoXY(F1.vertices[ver]));
    }
    return;
}
}
