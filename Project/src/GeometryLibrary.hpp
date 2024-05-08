#pragma once

#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;

namespace Geometry {
struct Fracture{
    unsigned int idFrac;
    unsigned int numVertices;
    vector <Vector3d> vertices={}; //coordinate dei vertici della frattura
    vector <unsigned int> passingTraces={}; //vettore con gli id delle tracce passanti per la frattura corrente
    vector <unsigned int> notPassingTraces={};

    Fracture()=default;

    bool checkFractureEdges(double tol);
};
}

