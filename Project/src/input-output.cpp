#include "GeometryLibrary.hpp"
#include<iostream>
#include<sstream>
#include<fstream>
#include <Eigen/Eigen>
#include <vector>
#include <filesystem>
#include <fstream>

using namespace std;
using namespace Eigen;
using namespace GeometryLibrary;
namespace fs = filesystem;

bool readFractures(const string& fileName, vector<Fracture>& fractures, const double& tol){

    ifstream ifstr(fileName);
    if(ifstr.fail()){
        cerr << "errore nell'apertura del file" << endl;
        return false;
    }

    // togliamo il # di header
    string header;
    getline(ifstr, header);

    string line;
    char semicolon; //semicolon=;

    // prendiamo il numero di fratture e riserviamo la memoria
    getline(ifstr, line);
    istringstream convert(line);
    unsigned int numFractures;
    convert >> numFractures;
    fractures.reserve(numFractures);

    while(getline(ifstr,line)){ //finché ci sono righe
        unsigned int id, numVertices; //definiamo variabili
        vector<Vector3d> vertices = {}; //inizializziamo vettore dei vertici

        getline(ifstr, line);
        istringstream convert(line);
        convert >> id >> semicolon >> numVertices;

        // preparo la memoria per i vertici
        vertices.resize(numVertices);

        // tolgo il #
        getline(ifstr, line);

        //per ogni dimensione, per ogni vertici, salviamo le coordinate dei vertici
        for (unsigned int i=0; i<3; i++){ //3 dimensioni (i=0 componente x)
            getline(ifstr, line);
            istringstream convert2(line);

            for (unsigned int j=0; j<numVertices; j++){ //numero di vertici
                convert2 >> ((vertices[j])[i]);
                convert2 >> semicolon;
            }
        }

        // Assegnamo i dati alla frattura (utilizziamo il costruttore della frattura per generarla
        Fracture frac = Fracture(id, vertices, tol);

        fractures.push_back(frac);
    }
    return true;
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
        // Print Trace details for not Passing Traces
        for (const auto& trace : fracture.notPassingTraces) {
            outFile << trace.idTrace << "; " << trace.tips << "; " << trace.length << endl;
        }

        outFile << endl;
    }

    outFile.close();
}

void printFractureToDebug(const Fracture& fracture, const string& filename) {
    std::ofstream file(filename, std::ios_base::app);

    if (!file.is_open()) {
        std::cerr << "Unable to open file";
        return;
    }

    file << "# FractureId; NumVertices\n";
    file << fracture.idFrac << "; " << fracture.numVertices << "\n";
    file << "# Vertices\n";

    for (const auto& vertex : fracture.vertices) {
        file << std::scientific << vertex.x() << "; " << vertex.y() << "; " << vertex.z() << "\n";
    }

    file.close();
}


void printTraceToDebug(const Trace& trace, const string& filename) {
    std::ofstream outFile(filename, std::ios_base::app);

    if (!outFile.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }


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


    outFile.close();
}

void printPointToDebug(const Vector3d& point, const string& filename) {
    std::ofstream file(filename, std::ios_base::app);

    if (!file.is_open()) {
        std::cerr << "Unable to open file";
        return;
    }

    file << "# Point\n";
    file << std::scientific << point.x() << "; " << point.y() << "; " << point.z() << "\n";


    file.close();
}

void saveMeshToFile(const PolygonalMesh& mesh, unsigned int idFracture) {
    // Create folder structure
    string folderName = "polygonalMeshes/mesh" + to_string(idFracture);
    string fileNameCell0D = folderName + "/Cell0D.txt";
    string fileNameCell1D = folderName + "/Cell1D.txt";
    string fileNameCell2D = folderName + "/Cell2D.txt";

    // Create folder
    try {
        fs::create_directories(folderName);
    } catch (const exception& e) {
        cerr << "Error creating folder: " << e.what() << endl;
        return;
    }

    // Save Cell0D data
    ofstream cell0DFile(fileNameCell0D);

    cell0DFile << "Number of Cells0D: " << mesh.NumberCell0D << endl;

    cell0DFile << "Id;X;Y;Z" << endl;
    for (const auto& cell : mesh.cells0D) {
        cell0DFile << cell.id << "; " << setprecision(16) << scientific
                   << cell.coordinates.x() << "; "
                   << cell.coordinates.y() << "; "
                   << cell.coordinates.z()
                   << endl;
    }
    cell0DFile.close();

    // Save Cell1D data
    ofstream cell1DFile(fileNameCell1D);

    unsigned int counter1 = 0;
    for (unsigned int i = 0; i< mesh.NumberCell1D; i++) {
        if (mesh.cells1D[i].alive == true) counter1++;
    }

    cell1DFile << "Number of Cells1D: " << counter1 << endl;


    cell1DFile << "Id;Origin;End" << endl;
    for (const auto& cell : mesh.cells1D) {

        if(!cell.alive) continue;

        cell1DFile << cell.id << ";" << cell.start << ";" << cell.end << endl;
    }
    cell1DFile.close();

    // Save Cell2D data
    ofstream cell2DFile(fileNameCell2D);
    unsigned int counter2 = 0;
    for (unsigned int i = 0; i< mesh.NumberCell2D; i++) {
        if (mesh.cells2D[i].alive == true) counter2++;
    }
    cell2DFile << "Number of Cells2D: " << counter2 << endl;

    cell2DFile << "Id;NumVertices;Vertices;NumEdges;Edges" << endl;
    for (const auto& cell : mesh.cells2D) {

        if(!cell.alive) continue;

        cell2DFile << cell.id << ";" << cell.vertices.size() << ";";
        for (const auto& vertex : cell.vertices) {
            cell2DFile << vertex << ";";
        }
        cell2DFile << cell.edges.size() << ";";
        for (const auto& edge : cell.edges) {
            cell2DFile << edge << ";";
        }
        cell2DFile << endl;
    }
    cell2DFile.close();

    cout << "Mesh data saved successfully." << endl;
}



