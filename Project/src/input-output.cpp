#include "GeometryLibrary.hpp"
#include<iostream>
#include<sstream>
#include<fstream>
#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace Geometry;

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
        frac.normal = findNormal(vertices[0], vertices[1], vertices[2]);
        frac.lyingPlane = findLyingPlane(frac.normal, tol);


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
        cout << "Fratture generatrici: " << trace.idGenerator1 << " " << trace.idGenerator2 << endl;

        for (unsigned int i=0; i<trace.extremes.size(); i++){
            cout << "Estremo " << i+1 << ": (" << trace.extremes[i][0] << ", " << trace.extremes[i][1] << ", " << trace.extremes[i][2] << ")" << endl;
        }

        cout << endl;
    }
}

