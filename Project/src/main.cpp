
#include "GeometryLibrary.hpp"
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>

using namespace std;
using namespace Geometry;

int main(int argc, char **argv)
{
    double tolInput=stod(argv[1]);
    double tol=max(10*numeric_limits<double>::epsilon(), tolInput);

    vector<Fracture> vec;
    string path = "./DFN";
    bool flag=readFractures(path+"./FR3_data.txt",vec, tol);


    for (Fracture fr : vec){
        cout << "numero fratture " << vec.size() << endl;
        cout << "frattura1 " << " id " << fr.idFrac << " numero vertici " << fr.numVertices;
        for (unsigned int i=0; i<fr.numVertices; i++){
            cout << " vertice " << i << ": " << fr.vertices[i][0] << " " << fr.vertices[i][1] << " " << fr.vertices[i][2] << endl;
        }
    }

    return 0;
}
