#include "Utils.hpp"
#include "GeometryLibrary.hpp"
#include "SortingAlgorithm.hpp"
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>

using namespace std;
using namespace Eigen;
using namespace Geometry;
using namespace SortLibrary;


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

    // controllo se ogni lato della frattura 1 interseca il piano della frattura 2
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

        // Check if intersections[v] is already in trace.extremes
        if (find(trace.extremes.begin(), trace.extremes.end(), intersections[v]) == trace.extremes.end())
        {
            projectIntersection(projIntersection, F2, intersections[v]);

            // If not present, check if projIntersection is inside projVertices
            if (isPointIn2DPolygon(projIntersection, projVertices, tol)) {

                // If so, push intersections[v] into trace.extremes
                trace.extremes.push_back(intersections[v]);
            }
        }
    }
}

bool checkTraceTips(Fracture F, Trace T, double tol) {
    unsigned int v_size = F.vertices.size();

    // flagE1 dice se il primo estremo è stato trovato nei segmenti
    bool flagE1 = false;
    for (unsigned int v = 0; v < v_size; v++) {
        if (isPointOn3DSegment(T.extremes[0], F.vertices[v % v_size], F.vertices[(v+1) % v_size], tol))
        {
            // se lo trova lo segna true e esce, altrimenti continua a cercarlo
            flagE1 = true;
            break;
        }
    }
    // se non lo trova, vuol dire che l'estremo non è su un lato della frattura, quindi la traccia è non passante e ritorniamo true
    if (flagE1 == false)
        return true;

    // ripetiamo per l'altro estremo della traccia
    bool flagE2 = false;
    for (unsigned int v = 0; v < v_size; v++) {
        if (isPointOn3DSegment(T.extremes[0], F.vertices[v % v_size], F.vertices[(v+1) % v_size], tol))
        {
            flagE2 = true;
            break;
        }
    }
    if (flagE2 == false)
        return true;

    // se tutto va bene, e troviamo entrambi gli estremi su uno dei lati della frattura ritorniamo flase cioè passante
    return false;
}

void addTraceToFractures(Fracture& F1, Fracture& F2, Trace& trace, double tol) {


    if (checkTraceTips(F1, trace, tol) == false) {
        trace.tips = false;
        F1.passingTraces.reserve(1);
        F1.passingTraces.push_back(trace);
    }
    else {
        trace.tips = true;
        F1.notPassingTraces.reserve(1);
        F1.notPassingTraces.push_back(trace);
    }

    if (checkTraceTips(F2, trace, tol) == false) {
        trace.tips = false;
        F2.passingTraces.reserve(1);
        F2.passingTraces.push_back(trace);
    }
    else {
        trace.tips = true;
        F2.notPassingTraces.reserve(1);
        F2.notPassingTraces.push_back(trace);
    }

}

unsigned int findTraces(vector<Trace>& traces, vector<Fracture>& fractures, const double& tol) {
    unsigned int c = 0;

    for (unsigned int i = 0; i < fractures.size(); i++) {
        // j < i perche prendiamo solo la parte triangolare inferiore della matrice essendo che 2 fratture hanno la stessa traccia
        for (unsigned int j = 0; j < i; j++) {

            Trace trace;


            Fracture& F1 = fractures[i];
            Fracture& F2 = fractures[j];

            Vector3d n1 = F1.normal;
            Vector3d n2 = F2.normal;



            if (n1.cross(n2).norm() < tol && abs(n1.dot(F1.vertices[0] - F2.vertices[0])) < tol) {
                // le due fratture sono complanari
            }
            else {
                BoundingSphere sphere1 = computeBoundingSphere(F1.vertices);
                BoundingSphere sphere2 = computeBoundingSphere(F2.vertices);
                // Se le sfere non si intersecano skippa alle prossime fratture per ottimizzare
                if (!spheresIntersect(sphere1, sphere2))
                    continue;

                // le due fratture giaciono su piani diversi

                // insersechiamo ogni lato della frattura 1 con la frattura 2 e viceversa
                findIntersections(trace, F1, F2, tol);
                findIntersections(trace, F2, F1, tol);

                if (trace.extremes.size() > 1)
                {
                    trace.idTrace = c++;
                    trace.idGenerator1 = F1.idFrac;
                    trace.idGenerator2 = F2.idFrac;
                    trace.length = (trace.extremes[0] - trace.extremes[1]).norm();

                    addTraceToFractures(F1, F2, trace, tol);

                    traces.push_back(trace);
                }
            }
        }
    }

    return traces.size();
}

void sortTraces(vector<Fracture>& fractures) {
    for (unsigned int i = 0; i < fractures.size(); i++) {
        // Sort passingTraces
        sort(fractures[i].passingTraces.begin(), fractures[i].passingTraces.end(),
             [] (const Trace& a, const Trace& b) { return a.length > b.length; }
             );

        // Sort notPassingTraces
        sort(fractures[i].notPassingTraces.begin(), fractures[i].notPassingTraces.end(),
             [] (const Trace& a, const Trace& b) { return a.length > b.length; }
             );
    }
}





















