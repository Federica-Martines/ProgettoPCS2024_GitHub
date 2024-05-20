#include "src/Utils.hpp"
#include "src/input-output.hpp"
#include "src/GeometryLibrary.hpp"
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>

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
    // printFractures(fractures, expectedNumFractures);

    vector<Trace> traces = {};
    findTraces(traces, fractures, tol);
    printTraces(traces);




    return 0;
}
