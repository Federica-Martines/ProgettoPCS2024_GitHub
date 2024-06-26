#include "src/Utils.hpp"
#include "src/input-output.hpp"
#include "src/GeometryLibrary.hpp"
#include <fstream>
#include <string>
#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <algorithm>
#include "src/PolygonalMesh.hpp"

using namespace std;
using namespace GeometryLibrary;
using namespace PolygonalLibrary;


int main(int argc, char **argv)
{

    double tol = numeric_limits<double>::epsilon()*100;
    if(argc >= 2)
    {
        double tolInput=stod(argv[1]);
        tol=max(tol, tolInput);
    }

    //inizializziamo le fratture
    vector<Fracture> fractures = {};
    string path = "./DFN/FR3_data.txt";
    // string path = "./DFN/FR362_data.txt";

    readFractures(path, fractures, tol);

    vector<Trace> traces = {};
    findTraces(traces, fractures, tol);

    sortTraces(fractures);

    string outputPathTraces = "./traces.txt";
    std::ofstream fileTraces(outputPathTraces, std::ios_base::trunc);
    printTracesToFile(traces, outputPathTraces);

    string outputPathFractures = "./fractures.txt";
    std::ofstream fileFractures(outputPathFractures, std::ios_base::trunc);
    printFracturesToFile(fractures, outputPathFractures);


    //PARTE 2

    //le seguenti 6 righe di codice servono per stampare attraverso un codice python
    string outputPathDebug = "./debug.txt";
    std::ofstream fileDebug(outputPathDebug, std::ios_base::trunc);

    string outputPathPointsDebug = "./debug_points.txt";
    std::ofstream filePoint(outputPathPointsDebug, std::ios_base::trunc);

    string outputPathTracesDebug = "./debug_traces.txt";
    std::ofstream fileTracesDebug(outputPathTracesDebug, std::ios_base::trunc);

    for (Fracture& F : fractures) {

        vector<Trace> cuts = {};
        // creo  i tagli da fare concatenando le tracce passanti e poi le non passanti
        cuts.insert(cuts.end(), F.passingTraces.begin(), F.passingTraces.end());
        cuts.insert(cuts.end(), F.notPassingTraces.begin(), F.notPassingTraces.end());

        PolygonalMesh mesh = convertFractureToMesh(F, tol);
saveMeshToFile(mesh, F.idFrac);
        cutMeshCell2D(mesh, cuts, tol);



        cout << "Saved mesh: " << F.idFrac << endl;
    }

    return 0;
}








