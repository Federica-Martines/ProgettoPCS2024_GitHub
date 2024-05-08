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

void findTraces(const vector<Fracture> fractures, vector<Trace>& traces) {

    for (int i = 0; i < fractures.size(); ++i) {
        for (int j = i + 1; j < fractures.size(); ++j) {

            // Calcolo i vettori che generano la normale per la prima frattura:
            Vector3d u1 = fractures[i].vertices[2] - fractures[i].vertices[0];
            Vector3d v1 = fractures[i].vertices[1] - fractures[i].vertices[0];
            Vector3d norm1 = (u1.cross(v1)).normalized();

            // Calcolo i vettori che generano la normale per la seconda frattura:
            Vector3d u2 = fractures[j].vertices[2] - fractures[i].vertices[0];
            Vector3d v2 = fractures[j].vertices[1] - fractures[i].vertices[0];
            Vector3d norm2 = (u2.cross(v2)).normalized();

            // Calcolo la direzione della retta di intersezione:
            Vector3d tangent = norm1.cross(norm2);
        }
    }
}


}



