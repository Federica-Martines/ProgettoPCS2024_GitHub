#include "src/Utils.hpp"
#include "src/input-output.hpp"
#include "src/GeometryLibrary.hpp"
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <vector>
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
    string path = "./DFN/FR10_data.txt";
    // string path = "./DFN/FR362_data.txt";

    unsigned int expectedNumFractures = readFractures(path, fractures, tol);

    vector<Trace> traces = {};
    findTraces(traces, fractures, tol);

    sortTraces(fractures);


    vector<Fracture> cuttedFractures;
    for (Fracture& frac : fractures) {

        deque<Trace> cuts = {};
        cuts.insert(cuts.end(), frac.passingTraces.begin(), frac.passingTraces.end());
        cuts.insert(cuts.end(), frac.notPassingTraces.begin(), frac.notPassingTraces.end());

        cuttingFracture(cuttedFractures, frac, cuts, tol);
    }


    // printFractures(fractures, expectedNumFractures);
    // printTraces(traces);

    string outputPathTraces = "./traces.txt";
    printTracesToFile(traces, outputPathTraces);

    string outputPathFractures = "./fractures.txt";
    printFracturesToFile(fractures, outputPathFractures);


    return 0;
}
