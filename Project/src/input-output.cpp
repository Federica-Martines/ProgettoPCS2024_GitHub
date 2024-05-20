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

void printTraces(vector<Trace> traces) {

    for (unsigned int j = 0; j < traces.size(); j++){
        Trace trace = traces[j];

        cout << "Traccia: " << trace.idTrace << endl;
        cout << "Fratture generatrici: " << trace.idGenerator1 << " " << trace.idGenerator2 << endl;
        cout << "Lunghezza: " << trace.length << endl;

        if(trace.tips == false) {
            cout << "La frattura è passante" << endl;
        }
        else {
             cout << "La frattura è NON passante" << endl;
        }

        for (unsigned int i=0; i<trace.extremes.size(); i++){
            cout << "Estremo " << i+1 << ": (" << trace.extremes[i][0] << ", " << trace.extremes[i][1] << ", " << trace.extremes[i][2] << ")" << endl;
        }

        cout << endl;
    }
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

        printTraces(fracture.passingTraces);
        printTraces(fracture.notPassingTraces);

        cout << endl;
    }
}

void printTracesToFile(const vector<Trace>& traces, const string& filename) {
    ofstream outFile(filename);

    if (!outFile.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    // Print the number of traces
    outFile << "# Number of Traces\n";
    outFile << traces.size() << endl;

    // Print each trace in the specified format
    outFile << "# TraceId; FractureId1; FractureId2; X1; Y1; Z1; X2; Y2; Z2\n";
    for (const auto& trace : traces) {
        if (trace.extremes.size() >= 2) {
            const Vector3d& start = trace.extremes[0];
            const Vector3d& end = trace.extremes[1];
            outFile << trace.idTrace << "; "
                    << trace.idGenerator1 << "; "
                    << trace.idGenerator2 << "; "
                    << start.x() << "; "
                    << start.y() << "; "
                    << start.z() << "; "
                    << end.x() << "; "
                    << end.y() << "; "
                    << end.z() << endl;
        }
    }

    outFile.close();
}

void printFracturesToFile(const vector<Fracture>& fractures, const string& filename) {
    ofstream outFile(filename);

    if (!outFile.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    for (const auto& fracture : fractures) {
        // Print Fracture ID and Number of Traces
        outFile << "# FractureId; NumTraces\n";
        outFile << fracture.idFrac << "; " << fracture.passingTraces.size() + fracture.notPassingTraces.size() << endl;

        // Print Trace details for Passing Traces
        outFile << "# TraceId; Tips; Length\n";
        for (const auto& trace : fracture.passingTraces) {
            outFile << trace.idTrace << "; " << trace.tips << "; " << trace.length << endl;
        }
        for (const auto& trace : fracture.notPassingTraces) {
            outFile << trace.idTrace << "; " << trace.tips << "; " << trace.length << endl;
        }

        outFile << endl;
    }

    outFile.close();
}




