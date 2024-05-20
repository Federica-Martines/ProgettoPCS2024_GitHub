#include "Utils.hpp"
#include "GeometryLibrary.hpp"
#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace Geometry;


bool checkSegmentIntersection(vector<Vector3d>& intersections, const Vector3d planeNormal, Vector3d planePoint, Vector3d a, Vector3d b, double tol) {

    Vector3d direction = (b - a).normalized();
    double value1 = planeNormal.dot(a-planePoint);
    double value2 = planeNormal.dot(b-planePoint);

    if (abs(value1) < tol){
        intersections.push_back(a);
        return true;
    }
    else if (abs(value2) < tol){
        intersections.push_back(b);
        return true;
    }
    else if (value1*value2 > tol) {
        // il segmento è completamente sopra o sotto il piano e non c'è intersezione
        return false;
    }
    else if (value1*value2 < tol){
        // il segmento attraversa il piano
        double t = planeNormal.dot(planePoint - a) / planeNormal.dot(direction);
        Vector3d intersection =  a + direction * t;

        intersections.push_back(intersection);
        return true;
    }
    return false;
}


void findIntersections(Trace& trace, Fracture F1, Fracture F2, double tol)
{
    vector<Vector3d> intersections;
    intersections.reserve(F1.vertices.size());

    for (unsigned int v = 0; v < F1.vertices.size()-1; v++ ) {
        checkSegmentIntersection(intersections, F2.normal, F2.vertices[0], F1.vertices[v], F1.vertices[v+1], tol);
    }
    checkSegmentIntersection(intersections, F2.normal, F2.vertices[0],  F1.vertices[0], F1.vertices[F1.vertices.size()-1], tol);

    // per ogni intersezione controlliamo che sia interna alla frattura, proiettando su un piano e usando il ray casting algorithm

    vector<Vector2d> projVertices;
    projVertices.reserve(F2.vertices.size());
    projectVertices(projVertices, F2);

    for (unsigned int v = 0; v < intersections.size(); v++ ) {
        Vector2d projIntersection;
        projectIntersection(projIntersection, F2, intersections[v]);

        // Check if intersections[v] is already in trace.extremes
        if (find(trace.extremes.begin(), trace.extremes.end(), intersections[v]) == trace.extremes.end())
        {
            // If not present, check if projIntersection is inside projVertices
            if (isPointIn2DPolygon(projIntersection, projVertices, tol)) {

                // If so, push intersections[v] into trace.extremes
                trace.extremes.push_back(intersections[v]);
            }
        }
    }
}


unsigned int findTraces(vector<Trace>& traces, vector<Fracture> fractures, const double& tol) {
    unsigned int c = 0;
    for (unsigned int i = 0; i < fractures.size(); i++) {
        // j < i perche prendiamo solo la parte triangolare inferiore della matrice essendo che 2 fratture hanno la stessa traccia
        for (unsigned int j = 0; j < i; j++) {

            Trace trace;


            Fracture F1 = fractures[i];
            Fracture F2 = fractures[j];

            Vector3d n1 = F1.normal;
            Vector3d n2 = F2.normal;

            if (abs(n1.cross(n2).norm()) < tol && abs(n1.dot(F1.vertices[0] - F2.vertices[0])) < tol) {
                // le due fratture sono complanari
            }
            else {
                // le due fratture giaciono su piani diversi

                // insersechiamo ogni lato della frattura 1 con la frattura 2 e viceversa
                findIntersections(trace, F1, F2, tol);
                findIntersections(trace, F2, F1, tol);

                if (trace.extremes.size() > 1)
                {
                    trace.idTrace = c++;
                    trace.idGenerator1 = F1.idFrac;
                    trace.idGenerator2 = F2.idFrac;
                    traces.push_back(trace);
                }
            }
        }
    }

    return traces.size();
}





















