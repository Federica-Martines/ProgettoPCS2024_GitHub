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




void cutMeshCell2D(PolygonalMesh& mesh, vector<Trace> cuts, double tol)
{
    for (Trace& cut : cuts)
    {

        bool firstIteration = true;
        Cell2D cell2D;
        double alpha = numeric_limits<double>::min();
        vector<unsigned int> intersections = {numeric_limits<unsigned int>::max(), numeric_limits<unsigned int>::max()};
        vector<unsigned int> old_intersections = {numeric_limits<unsigned int>::max(), numeric_limits<unsigned int>::max()};
        unsigned int intersectionCount = 0;
        vector<int> neigh_intersections = {numeric_limits<int>::min(), numeric_limits<int>::min()};

        // faccio tagli fino a quando tutti i vicini non sono stati esaminati
        while (neigh_intersections[0] != -1 || neigh_intersections[1] != -1) {
            // la prima volta cerco la cella2D dove è localizzato il taglio nella mesh
            if (firstIteration) {
                // trovo la cella a cui il taglio appartiene
                if (!findCellContainingPoint(cell2D, mesh, cut.extremes[0], tol))
                {
                    cout << "Cella non tovata per il taglio: " << cut.idTrace << endl;
                }
                firstIteration = false;
            }
            else {
                int neighbourId = numeric_limits<int>::min();
                for (unsigned int i = 0; i < 2; i++) {
                    if (neigh_intersections[i] == -1)
                        continue;
                    else
                    {
                        neighbourId = neigh_intersections[i];
                    }
                }
                if (neighbourId == numeric_limits<int>::min() || !mesh.cells2D[neighbourId].alive)
                    throw runtime_error("Vicino non trovato");

                cell2D = mesh.cells2D[neighbourId];

                old_intersections = intersections;
                intersections = {numeric_limits<unsigned int>::max(), numeric_limits<unsigned int>::max()};
                intersectionCount = 0;
                neigh_intersections = {numeric_limits<int>::min(), numeric_limits<int>::min()};

            }

            vector<unsigned int> vertices = cell2D.vertices;
            vector<unsigned int> edges = cell2D.edges;
            unsigned int numEdges = cell2D.edges.size();

            // trovo le intersezioni
            for (unsigned int nEdge = 0; nEdge < numEdges; nEdge++)
            {

                if (intersections[0] != numeric_limits<unsigned int>::max() && intersections[1] != numeric_limits<unsigned int>::max())
                    break;

                Vector3d intersection;
                Cell1D edge = mesh.cells1D[edges[nEdge]];
                double beta;
                int position = findLineSegmentIntersection(intersection, mesh, alpha, beta, cut, edge, tol);

                if (position == -1)
                    continue;

                if (old_intersections[0] != numeric_limits<unsigned int>::max())
                {
                    if (areVectorsEqual(intersection, mesh.cells0D[old_intersections[0]].coordinates, tol))
                    {
                        if(intersectionCount == 1)
                            if(areVectorsEqual(intersection, mesh.cells0D[intersections[0]].coordinates, tol))
                                continue;

                        intersections[intersectionCount] = old_intersections[0];
                        neigh_intersections[intersectionCount++] = -1;
                        continue;
                    }
                }
                if (old_intersections[1] != numeric_limits<unsigned int>::max())
                {
                    if (areVectorsEqual(intersection, mesh.cells0D[old_intersections[1]].coordinates, tol))
                    {
                        if(intersectionCount == 1)
                            if(areVectorsEqual(intersection, mesh.cells0D[intersections[0]].coordinates, tol))
                                continue;

                        intersections[intersectionCount] = old_intersections[1];
                        neigh_intersections[intersectionCount++] = -1;
                        continue;
                    }
                }

                unsigned int id_new_points = numeric_limits<unsigned int>::max();
                if (position == 1)
                {
                    splitEdge(id_new_points, cell2D, mesh, edge, intersection);
                    if(alpha <= tol || alpha >= 1-tol)
                        neigh_intersections[intersectionCount] = -1;
                    else
                        neigh_intersections[intersectionCount] = findNeighbour(mesh, cell2D.id, edge.id);
                }
                else if (position == 0) {
                    unsigned int edgeOppositeId = numeric_limits<unsigned int>::max();
                    // trovo l'indice dell'opposto con cui si interseca con position 0
                    for (unsigned int eS = 0; eS < cell2D.edges.size(); eS++) {
                        //faccio eS-1 perchè in senso antiorario sarà quello prima
                        if (edge.id == cell2D.edges[eS]){
                            edgeOppositeId = cell2D.edges[(eS-1) % cell2D.edges.size()];
                            break;
                        }
                    }
                    if (edgeOppositeId == numeric_limits<unsigned int>::max()) {
                        throw runtime_error("Vicino in position == 0 non trovato");
                    }
                    Cell1D edgeOpposite = mesh.cells1D[edgeOppositeId];
                    unsigned int nOpposite = findNeighbour(mesh, cell2D.id, edgeOppositeId);
                    neigh_intersections[intersectionCount] = nOpposite;

                    if (abs(beta) < tol)
                    {
                        id_new_points = edge.start;
                    }
                    else
                    {
                        id_new_points = edge.end;
                    }

                }
                else
                    throw runtime_error("invalid position");

                intersections[intersectionCount++] = id_new_points;

            }

            // costruisco la nuova cella2d
            generateCell2D(mesh, cell2D.id, intersections[0], intersections[1]);

        }
    }

}

