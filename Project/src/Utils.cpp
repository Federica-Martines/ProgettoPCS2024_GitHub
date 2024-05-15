#include "Utils.hpp"
#include "GeometryLibrary.hpp"
#include<iostream>
#include<sstream>
#include<fstream>
#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace Geometry;

namespace Utils {
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

void printFractures(vector<Fracture> fractures, unsigned int expectedNumFractures) {
    cout << "Numero previsto di fratture: " << expectedNumFractures << endl;
    cout << "Numero di fratture: " << fractures.size() << endl << endl;

    for (unsigned int j = 0; j < fractures.size(); j++){
        Fracture fracture = fractures[j];

        cout << "Frattura: " << fracture.idFrac << endl;
        cout << "Numero vertici: " << fracture.numVertices << endl;

        for (unsigned int i=0; i<fracture.numVertices; i++){
            cout << "Vertice " << i << ": (" << fracture.vertices[i][0] << ", " << fracture.vertices[i][1] << ", " << fracture.vertices[i][2] << ")" << endl;
        }

        cout << endl;
    }
}

void printTraces(vector<Trace> traces) {
    cout << "Numero di fratture: " << traces.size() << endl << endl;

    for (unsigned int j = 0; j < traces.size(); j++){
        Trace trace = traces[j];

        cout << "Traccia: " << trace.idTrace << endl;
        cout << "Tracce generatrici:" << endl << trace.idGenerator1 << " " << trace.idGenerator2 << endl;

        for (unsigned int i=0; i<trace.extremes.size(); i++){
            cout << "Estremo " << i+1 << ": ("
                                           "" << trace.extremes[i][0] << ", " << trace.extremes[i][1] << ", " << trace.extremes[i][2] << ")" << endl;
        }

        cout << endl;
    }
}
}

namespace Geometry {
// given 3 points not aligned returns the normal to the plane passing in the 3 points
Vector3d findNormal(const Vector3d p1, const Vector3d p2, const Vector3d p3) {

    // Calcolo i vettori che generano la normale
    Vector3d u1 = p3 - p1;
    Vector3d v1 = p2 - p1;
    return u1.cross(v1).normalized();
}

bool checkSegmentIntersection(const Vector3d planeNormal, Vector3d planePoint, Vector3d s1, Vector3d s2, double tol) {

    double value1 = planeNormal.dot(s1-planePoint);
    double value2 = planeNormal.dot(s2-planePoint);

    if (value1*value2 > tol) {
        // il segmento è completamente sopra o sotto il piano e non c'è intersezione
        return false;
    }
    else if (value1*value2 < tol){
        // il segmento attraversa il piano
        return true;
    }
    else {
        // il segmento tocca il piano con uno o entrambi i punti
        return true;
    }
}

bool addLineIntersection(vector<Vector3d>& intersections, Vector3d planeNormal, Vector3d planePoint, Vector3d p1, Vector3d p2, double tol) {
    Vector3d direction = (p2 - p1).normalized();

    // se la traccia è parallela al piano controllo se uno dei due punti tocca il piano
    if (abs(planeNormal.dot(direction)) < tol) {
        double value1 = planeNormal.dot(p1-planePoint);
        double value2 = planeNormal.dot(p2-planePoint);
        if (abs(value1) < tol){
            intersections.push_back(p1);
            return true;
        }
        if (abs(value2) < tol){
            intersections.push_back(p2);
            return true;
        }

        return false;
    }

    double t = planeNormal.dot(planePoint - p1) / planeNormal.dot(direction);
    Vector3d intersection =  p1 + direction * t;

    intersections.push_back(intersection);
    return true;
}


unsigned int findTraces(vector<Trace>& traces, vector<Fracture> fractures, const double& tol) {
    Trace trace;

    for (unsigned int i = 0; i < fractures.size(); i++) {
        // j < i perche prendiamo solo la parte triangolare inferiore della matrice essendo che 2 fratture hanno la stessa traccia
        for (unsigned int j = 0; j < i; j++) {

            Fracture F1 = fractures[i];
            Fracture F2 = fractures[j];

            Vector3d n1 = findNormal(F1.vertices[0], F1.vertices[1], F1.vertices[2]);
            Vector3d n2 = findNormal(F2.vertices[0], F2.vertices[1], F2.vertices[2]);

            if (abs(n1.cross(n2).norm()) < tol && abs(n1.dot(F1.vertices[0] - F2.vertices[0])) < tol) {
                // le due fratture sono complanari
            }
            else {
                // le due fratture giaciono su piani diversi

                // insersechiamo ogni lato della frattura i con la frattura j
                vector<Vector3d> intersections;
                intersections.reserve(F1.vertices.size());

                for (unsigned int v = 0; v < F1.vertices.size()-1; v++ ) {
                    if(checkSegmentIntersection(n2, F2.vertices[0], F1.vertices[v], F1.vertices[v+1], tol)) {
                        Vector3d intersection;

                        addLineIntersection(intersections, n2, F2.vertices[0], F1.vertices[v], F1.vertices[v+1], tol);
                    }
                }
                if(checkSegmentIntersection(n2, F2.vertices[0], F1.vertices[0], F1.vertices[F1.vertices.size()-1], tol)) {
                    Vector3d intersection;

                    addLineIntersection(intersections, n2, F2.vertices[0],  F1.vertices[0], F1.vertices[F1.vertices.size()-1], tol);
                }


                // per ogni intersezione controlliamo che sia interna alla frattura, proiettando su un piano e usando il ray casting algorithn
                for (unsigned int v = 0; v < intersections.size(); v++ ) {
                    Vector2d projIntersection;
                    vector<Vector2d> projVertices;
                    projVertices.reserve(F1.vertices.size());

                    projectIntVer(projIntersection, projVertices, F1, n1, intersections[v], tol);

                    if (isPointIn2DPolygon(projIntersection, projVertices)) {
                        trace.extremes.push_back(intersections[v]);
                    }
                }

                if (trace.extremes.size() > 1)
                {
                    trace.idGenerator1 = F1.idFrac;
                    trace.idGenerator2 = F2.idFrac;
                    traces.push_back(trace);
                }
            }
        }
    }

    return traces.size();
}



}
