#include "src/Utils.hpp"
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

    vector<Fracture> vec = {};
    string path = "./DFN/FR3_data.txt";
    unsigned int numFractures = readFractures(path, vec, tol);

    cout << "Numero previsto di fratture: " << numFractures << endl;
    cout << "Numero di fratture: " << vec.size() << endl << endl;

    for (unsigned int j = 0; j < numFractures; j++){
        Fracture frac = vec[j];

        cout << "Frattura: " << frac.idFrac << endl;
        cout << "Numero vertici: " << frac.numVertices << endl;

        for (unsigned int i=0; i<frac.numVertices; i++){
            cout << "Vertice " << i << ": " << frac.vertices[i][0] << " " << frac.vertices[i][1] << " " << frac.vertices[i][2] << endl;
        }

        cout << endl;
        cout << "ciao";
    }

    return 0;
}
