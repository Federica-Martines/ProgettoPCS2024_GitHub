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

unsigned int readFractures(const string& fileName, vector<Fracture>& vec, const double& tol){
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
    vec.reserve(numFractures);

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
            vec.push_back(frac);
    }
    return numFractures;
}
}



