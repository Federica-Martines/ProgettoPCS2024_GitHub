#include "src/Utils.hpp"
#include "src/input-output.hpp"
#include "src/GeometryLibrary.hpp"
#include <fstream>
#include <string>
#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <algorithm>
#include <deque>

using namespace std;
using namespace Geometry;

int main(int argc, char **argv)
{

    double tol = 10*numeric_limits<double>::epsilon();
    if(argc >= 2)
    {
        double tolInput=stod(argv[1]);
        tol=max(tol, tolInput);
    }

    vector<Fracture> fractures = {};
    string path = "./DFN/FR200_data.txt";
    // string path = "./DFN/FR362_data.txt";

    unsigned int expectedNumFractures = readFractures(path, fractures, tol);

    vector<Trace> traces = {};
    findTraces(traces, fractures, tol);

    sortTraces(fractures);


    string outputPathTraces = "./traces.txt";
    std::ofstream fileTraces(outputPathTraces, std::ios_base::trunc);
    //printTracesToFile(traces, outputPathTraces);

    string outputPathFractures = "./fractures.txt";
    std::ofstream fileFractures(outputPathFractures, std::ios_base::trunc);
    //printFracturesToFile(fractures, outputPathFractures);


    string outputPathDebug = "./debug.txt";
    std::ofstream fileDebug(outputPathDebug, std::ios_base::trunc);

    string outputPathPointsDebug = "./debug_points.txt";
    std::ofstream filePoint(outputPathPointsDebug, std::ios_base::trunc);



    vector<Fracture> cuttedFractures;
    for (Fracture& frac : fractures) {

        deque<Trace> cuts = {};
        // creo  i tagli da fare concatenando le tracce passanti e poi le non passanti
        cuts.insert(cuts.end(), frac.passingTraces.begin(), frac.passingTraces.end());
        cuts.insert(cuts.end(), frac.notPassingTraces.begin(), frac.notPassingTraces.end());

        cuttingFracture(cuttedFractures, frac, cuts, tol);
        cout << "Cutted fracture: " << frac.idFrac << endl;
    }

    string outputPathDebugFractures = "./log_fractures.txt";
    std::ofstream fileDebugFracture(outputPathDebugFractures, std::ios_base::trunc);

    for (Fracture& F : cuttedFractures) {
        printFractureToDebug(F, outputPathDebugFractures);
    }

    //printFractures(fractures, expectedNumFractures);
    // print    Traces(traces);




    return 0;
}
