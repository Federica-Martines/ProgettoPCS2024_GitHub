#include "Utils.hpp"
#include "GeometryLibrary.hpp"
#include<iostream>
#include<sstream>
#include<fstream>
#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;

namespace Geometry {

unsigned int readFractures(const string& fileName, vector<Fracture>& fractures, const double& tol){

    ifstream ifstr(fileName);
    if(ifstr.fail()){
        cerr << "errore nell'apertura del file" << endl;
        return false;
    }

    // togliamo il # di header
    string header;
    getline(ifstr, header);

    string line;
    char semicolon;

    // prendiamo il numero di fratture e riserviamo la memoria
    getline(ifstr, line);
    istringstream convert(line);
    unsigned int numFractures;
    convert >> numFractures;
    fractures.reserve(numFractures);

    while(getline(ifstr,line)){
        Fracture frac;
        unsigned int id, numVertices;
        vector<Vector3d> vertices = {};

        getline(ifstr, line);
        istringstream convert(line);
        convert >> id >> semicolon >> numVertices;

        // preparo la memoria per i vertici
        vertices.resize(numVertices);

        // tolgo il #
        getline(ifstr, line);


        for (unsigned int i=0; i<3; i++){ //3 dimensioni (i=0 componente x)
            getline(ifstr, line);
            istringstream convert2(line);

            for (unsigned int j=0; j<numVertices; j++){ //numero di vertici
                convert2 >> ((vertices[j])[i]);
                convert2 >> semicolon;
            }
        }

        // Assegnamo i dati alla frattura
        frac.idFrac = id;
        frac.numVertices = numVertices;
        frac.vertices = vertices;


        // Eseguo un controllo sulle tracce per verificare che non abbiano lati null
        if (frac.checkFractureEdges(tol))
            fractures.push_back(frac);
    }
    return numFractures;
}

// given 3 points not aligned returns the normal to the plane passing in the 3 points
Vector3d findNormal(const Vector3d p1, const Vector3d p2, const Vector3d p3) {

    // Calcolo i vettori che generano la normale
    Vector3d u1 = p3 - p1;
    Vector3d v1 = p2 - p1;
    return u1.cross(v1).normalized();
}

bool checkSegmentIntersection(const Vector3d n, Vector3d P, Vector3d s1, Vector3d s2) {

    double value1 = n.dot(s1-P);
    double value2 = n.dot(s2-P);

    if (value1*value2 > 0) {
        // il segmento è completamente sopra o sotto il piano e non c'è intersezione
        return false;
    }
    else if (value1*value2 < 0){
        // il segmento attraversa il piano
        return true;
    }
    else {
        // il segmento tocca il piano con uno o entrambi i punti
        return true;
    }
}

bool lineIntersection(Vector3d& intersection, Vector3d planePoint, Vector3d planeNormal, Vector3d p1, Vector3d p2) {
    Vector3d direction = (p2 - p1).normalized();

    if (planeNormal.dot(direction) == 0) {
        return false;
    }

    double t = planeNormal.dot(planePoint - p1) / planeNormal.dot(direction);
    intersection =  p1 + direction * t;

    return true;
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



unsigned int findTraces(vector<Fracture>& fractures, vector<Trace>& traces, const double& tol) {
    Trace trace;

    for (unsigned int i = 0; i < fractures.size(); i++) {
        for (unsigned int j = 0; j < fractures.size(); j++) {

            Fracture F1 = fractures[i];
            Fracture F2 = fractures[j];

            Vector3d n1 = findNormal(F1.vertices[0], F1.vertices[1], F1.vertices[2]);
            Vector3d n2 = findNormal(F2.vertices[0], F2.vertices[1], F2.vertices[2]);

            if (n1.cross(n2).norm() == 0  && n1.dot(F1.vertices[0] - F2.vertices[0]) == 0) {
                // le due fratture sono complanari
            }
            else {
                // le due fratture giaciono su piani diversi

                // insersechiamo ogni lato della frattura i con la frattura j
                vector<Vector3d> intersections;
                intersections.reserve(F1.vertices.size());

                for (unsigned int v = 0; v < F1.vertices.size(); v++ ) {

                    if(checkSegmentIntersection(n2, F2.vertices[0], F1.vertices[v], F1.vertices[v+1])) {
                        Vector3d intersection;

                        lineIntersection(intersection, n2, F2.vertices[0], F1.vertices[v], F1.vertices[v+1]);

                        intersections.push_back(intersection);
                    }
                }

                vector<Vector2d> projIntersections;
                projIntersections.reserve(intersections.size());
                // controlliamo se le intersezioni sono interne alle fratture e se si creiamo la traccia
                for (unsigned int v = 0; v < intersections.size(); v++ ) {

                    Vector3d eZ = {0,0,1};

                    vector<Vector2d> projVertices;
                    projVertices.reserve(F1.vertices.size());

                    // controlliamo se il piano contenente F1 è ortogonale al piano XY
                    if (n1.dot(eZ) == 0) {

                        projIntersections[v] = projectOntoXZ(intersections[v]);
                        for (unsigned int ver = 0; ver < F1.vertices.size(); ver++) {
                            projVertices.push_back(projectOntoXZ(F1.vertices[ver]));
                        }
                    }
                    else {
                        projIntersections[v] = projectOntoXY(intersections[v]);
                        for (unsigned int ver = 0; ver < F1.vertices.size(); ver++) {
                            projVertices.push_back(projectOntoXY(F1.vertices[ver]));
                        }
                    }

                    if (isPointIn2DPolygon(projIntersections[v], projVertices)) {
                        trace.vertices.push_back(intersections[v]);
                    }

                }

                if (trace.vertices.size() > 0)
                {
                    trace.idFracture1 = F1.idFrac;
                    trace.idFracture2 = F2.idFrac;
                    traces.push_back(trace);
                }
            }

        }
    }

    return traces.size();
}



}
