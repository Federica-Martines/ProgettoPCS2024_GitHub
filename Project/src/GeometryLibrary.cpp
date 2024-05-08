#include "GeometryLibrary.hpp"
#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

namespace Geometry {

bool Fracture::checkFractureEdges(double tol){

    //controlliamo che non ci siano lati di lunghezza nulla.
    for (unsigned int k=0; k<this->numVertices-1; k++){

        //differenza tra due vertici consecutivi
        Vector3d edge = (this->vertices)[k]-(this->vertices)[k+1];
        if (edge[0]*edge[0]+edge[1]*edge[1]+edge[2]*edge[2]<tol*tol){
            cerr << "la frattura " << this->idFrac << " ha lati di lunghezza nulla" << endl;
            //costruttore particolare per differenziare le fratture con problemi
            return false;
        }
    }
    Vector3d edgeF = (this->vertices)[0]-(this->vertices)[this->numVertices-1]; //faccio lo stesso per il primo e l'ultimo vertice
    if (edgeF[0]*edgeF[0]+edgeF[1]*edgeF[1]+edgeF[2]*edgeF[2]<tol*tol){
        cerr << "la frattura " << this->idFrac << " ha lati di lunghezza nulla" << endl;
        return false;
    }
    return true;
};
}
