#include "Utils.hpp"
#include <iostream>
#include "GeometryLibrary.hpp"
#include "input-output.hpp"
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>
#include <deque>

using namespace std;
using namespace Eigen;
using namespace GeometryLibrary;
using namespace PolygonalLibrary;

// s1, s2 sono gli estremi di ogni lato della frattura. il punto e la normale sono del piano dell'altra frattura
bool checkSegmentPlaneIntersection(vector<Vector3d>& intersections, const Vector3d planeNormal, Vector3d planePoint, Vector3d s1, Vector3d s2, double tol) {

    Vector3d direction = (s2 - s1).normalized(); //normalizzato così ottengo il versore
    double value1 = planeNormal.dot(s1-planePoint);
    double value2 = planeNormal.dot(s2-planePoint);

    // Se il punto è sul piano value1 restituisce zero e quindi si intersecano
    if (abs(value1) < tol){
        intersections.push_back(s1);
        return true;
    }
    else if (abs(value2) < tol){
        intersections.push_back(s2);
        return true;
    }
    else if (value1*value2 > tol) {
        // il segmento è completamente sopra o sotto il piano e non c'è intersezione
        return false;
    }
    else if (value1*value2 < tol){
        // il segmento attraversa il piano
        double t = planeNormal.dot(planePoint - s1) / planeNormal.dot(direction);

        Vector3d intersection =  s1 + direction * t;

        intersections.push_back(intersection);
        return true;
    }
    return false;
}

void findIntersections(Trace& trace, Fracture F1, Fracture F2, double tol)
{
    vector<Vector3d> intersections;
    intersections.reserve(F1.vertices.size());

    // controllo se ogni lato della frattura 1 interseca il piano della frattura 2 (successivamente dovrò controllare che le interesezioni siano interne alla frattura (della quale considero il piano)
    for (unsigned int v = 0; v < F1.vertices.size()-1; v++ ) {
        checkSegmentPlaneIntersection(intersections, F2.normal, F2.vertices[0], F1.vertices[v], F1.vertices[v+1], tol);
    }
    checkSegmentPlaneIntersection(intersections, F2.normal, F2.vertices[0],  F1.vertices[0], F1.vertices[F1.vertices.size()-1], tol);

    // per ogni intersezione controlliamo che sia interna alla frattura, proiettando su un piano e usando il ray casting algorithm
    vector<Vector2d> projVertices;
    projVertices.reserve(F2.vertices.size());
    projectVertices(projVertices, F2);

    for (unsigned int v = 0; v < intersections.size(); v++ ) {
        Vector2d projIntersection;

        // Controlla se intersections[v] è già in trace.extremes
        if (find(trace.extremes.begin(), trace.extremes.end(), intersections[v]) == trace.extremes.end()) //se la find va fino in fondo trova end
        {   //se non lo trova ritorna end, e controlla che sia interno
            projectIntersection(projIntersection, F2, intersections[v]);

            // Se non è presente, controlla se projIntersection è dentro projVertices (la frattura proeittato)
            if (isPointIn2DPolygon(projIntersection, projVertices, tol)) {

                // Se il punto è interno, push intersections[v] in trace.extremes
                trace.extremes.push_back(intersections[v]);
            }
        }
    }
}

bool checkTraceTips(Fracture F, Trace T, double tol) {
    unsigned int v_size = F.vertices.size();

    // flagE1 dice se il primo estremo è stato trovato nei segmenti (su uno dei lati della frattura)
    bool flagE1 = false;
    for (unsigned int v = 0; v < v_size; v++) {
        if (isPointOn3DSegment(T.extremes[0], F.vertices[v % v_size], F.vertices[(v+1) % v_size], tol))
        {
            // se lo trova lo segna true e esce, altrimenti continua a cercarlo
            flagE1 = true;
            break; //smetti di fare il for
        }
    }
    // se non lo trova, vuol dire che l'estremo non è su un lato della frattura, quindi la traccia è non passante e ritorniamo true
    if (flagE1 == false)
        return true; //true significa NON passante

    // ripetiamo per l'altro estremo della traccia
    bool flagE2 = false;
    for (unsigned int v = 0; v < v_size; v++) {
        if (isPointOn3DSegment(T.extremes[1], F.vertices[v % v_size], F.vertices[(v+1) % v_size], tol))
        {
            flagE2 = true;
            break;
        }
    }
    if (flagE2 == false)
        return true;

    // se tutto va bene, e troviamo entrambi gli estremi su uno dei lati della frattura ritorniamo false cioè passante
    return false;
}

void addTraceToFractures(Fracture& F1, Fracture& F2, Trace& trace, double tol) {


    if (checkTraceTips(F1, trace, tol) == false) {
        trace.tips = false;
        F1.passingTraces.push_back(trace);
    }
    else {
        trace.tips = true;
        F1.notPassingTraces.push_back(trace);
    }

    if (checkTraceTips(F2, trace, tol) == false) {
        trace.tips = false;
        F2.passingTraces.push_back(trace);
    }
    else {
        trace.tips = true;
        F2.notPassingTraces.push_back(trace);
    }

}

//funzione : prima cose da riempire, poi i dati; in questo caso vogliamo rimpeire il vettore di tracce
unsigned int findTraces(vector<Trace>& traces, vector<Fracture>& fractures, const double& tol) {
    unsigned int idTraces = 0;

    for (unsigned int i = 0; i < fractures.size(); i++) {
        // j < i perche prendiamo solo la parte triangolare inferiore della matrice essendo che 2 fratture hanno la stessa traccia
        for (unsigned int j = 0; j < i; j++) {

            Trace trace;

            Fracture& F1 = fractures[i]; //& reference (accorcio il nome)
            Fracture& F2 = fractures[j];

            Vector3d n1 = F1.normal;
            Vector3d n2 = F2.normal;

            //creiamo due sfere
            BoundingSphere sphere1 = computeBoundingSphere(F1.vertices);
            BoundingSphere sphere2 = computeBoundingSphere(F2.vertices);
            // Se le sfere non si intersecano skippa alle prossime fratture per ottimizzare
            if (!spheresIntersect(sphere1, sphere2))
                continue;

            if (n1.cross(n2).norm() < tol && abs(n1.dot(F1.vertices[0] - F2.vertices[0])) < tol) {
                // le due fratture sono complanari
                cerr << "le fratture " << F1.idFrac << " e " << F2.idFrac << " sono complanari." << endl;
            }
            else {
                // le due fratture giacciono su piani diversi

                // insersechiamo ogni lato della frattura 1 con la frattura 2 e viceversa
                findIntersections(trace, F1, F2, tol);
                findIntersections(trace, F2, F1, tol);

                if (trace.extremes.size() > 1)
                {
                    trace.idTrace = idTraces++;
                    trace.idGenerator1 = F1.idFrac;
                    trace.idGenerator2 = F2.idFrac;
                    trace.length = (trace.extremes[0] - trace.extremes[1]).norm();

                    addTraceToFractures(F1, F2, trace, tol);

                    traces.push_back(trace); //traces è una lista
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
             ); //funzione inline dal [] in poi. non chiama lo stack frame. è la funzione di confronto

        // Sort notPassingTraces
        sort(fractures[i].notPassingTraces.begin(), fractures[i].notPassingTraces.end(),
             [] (const Trace& a, const Trace& b) { return a.length > b.length; }
             );
    }
}



void cutMeshCell2D(PolygonalMesh& mesh, vector<Trace> cuts, double tol) {

    for (Trace& cut : cuts) {

        // trovo la cella a cui il taglio appartiene
        Cell2D cell2D;
        if (!findCellContainingPoint(cell2D, mesh, cut.extremes[0], tol))
        {
            cout << "Cella non tovata per il taglio: " << cut.idTrace << endl;
        }


        vector<unsigned int> intersectionsBase;

        // cerchiamo le intersezioni
        for (unsigned int e = 0; e < cell2D.edges.size(); e++) {
            unsigned int edgeId = cell2D.edges[e];
            Cell1D edge = mesh.cells1D[edgeId];
            double alpha, beta;

            Vector3d intersectionBase;
            unsigned int intersectionBaseId;


            int position = findLineSegmentIntersection(intersectionBase, mesh, alpha, beta, cut, edge, tol);

            // se non c'è intersezione skippo lato
            if (position == -1) continue;
            // se c'è in mezzo al lato lo splitto creando un vertice e due nuovi lati
            if (position == 0) {
                if (round(beta) == 0) intersectionBaseId = edge.start;
                if (round(beta) == 1) intersectionBaseId = edge.end;
            }
            // IMPORTANTE faccio e++ perchè ho appena aggiunto un edge
            if (position == 1) {
                splitEdge(intersectionBaseId, cell2D, mesh, edge, intersectionBase);
                e++;

                // refresh della reference perchè facendo splitEdge modifico la mesh ma non la cella corrente
                cell2D = mesh.cells2D[cell2D.id];

            }
            intersectionsBase.push_back(intersectionBaseId);


            Cell2D neighbour = cell2D;
            Cell1D edgeNext = edge;
            Vector3d intersection = intersectionBase;
            unsigned int intersectionId = intersectionBaseId;
            Vector3d intersectionNext;
            unsigned int intersectionNextId;
            int positionNext;

            // faccio tagli ulteriori se alpha è compresa strettamente tra 0 e 1 e la traccia è non passante
            while (cut.tips == true && alpha < 1-tol  && alpha > tol) {
                // trovo il vicino
                unsigned int n = findNeighbour(mesh, neighbour.id, edgeNext.id);
                neighbour = mesh.cells2D[n];


                // trovo la prossima intersezione
                for (unsigned int nEdge = 0; nEdge < neighbour.edges.size(); nEdge++) {
                    unsigned int edgeNextId = neighbour.edges[nEdge];
                    edgeNext = mesh.cells1D[edgeNextId];

                    positionNext = findLineSegmentIntersection(intersectionNext, mesh, alpha, beta, cut, edgeNext, tol);

                    // se ho trovato la stessa intersezione di prima, o non l'ho trovata skippo
                    if (areVectorsEqual(intersection, intersectionNext, tol) || positionNext == -1) continue;


                    if (positionNext == 1) {
                        splitEdge(intersectionNextId, neighbour, mesh, edgeNext, intersectionNext);
                        nEdge++;
                        neighbour = mesh.cells2D[neighbour.id];
                    }
                    if (positionNext == 0) {
                        if (round(beta) == 0) intersectionNextId = edge.start;
                        if (round(beta) == 1) intersectionNextId = edge.end;
                    }
                    break;
                }

                // costruisco la nuova cella2d
                generateCell2D(mesh, neighbour, intersectionId, intersectionNextId);

                intersection = intersectionNext;
                intersectionId = intersectionNextId;
                edge = edgeNext;
                position =  positionNext;
            }

            if (intersectionsBase.size() == 2) {
                break;
            }
        }

        if (intersectionsBase.size() == 2) {
            generateCell2D(mesh, cell2D, intersectionsBase[0], intersectionsBase[1]);
        } else {
            cout << "Error: Expected 2 intersections but found " << intersectionsBase.size() << " for cut: " << cut.idTrace << endl;
        }


    }

}
