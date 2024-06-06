#ifndef POLYGONALMESH_HPP
#define POLYGONALMESH_HPP
//PolygonalMesh.hpp

#pragma once

#include <iostream>
#include "Eigen/Eigen"
#include "GeometryLibrary.hpp"

using namespace std;
using namespace Eigen;
using namespace GeometryLibrary;

namespace PolygonalLibrary {

struct PolygonalMesh
{
    unsigned int NumberCell0D = 0; ///< number of Cell0D
    std::vector<unsigned int> Cell0DId = {}; ///< Cell0D id, size 1 x NumberCell0D
    std::vector<Vector3d> Cell0DCoordinates = {}; ///< Cell0D coordinates, size 2 x NumberCell0D (x,y)
    std::map<unsigned int, list<unsigned int>> Cell0DMarkers = {}; ///< Cell0D markers, size 1 x NumberCell0D (marker)

    unsigned int NumberCell1D = 0; ///< number of Cell1D
    std::vector<unsigned int> Cell1DId = {}; ///< Cell1D id, size 1 x NumberCell1D
    std::vector<Vector2i> Cell1DVertices = {}; ///< Cell1D vertices indices, size 2 x NumberCell1D (fromId,toId)
    std::map<unsigned int, list<unsigned int>> Cell1DMarkers = {}; ///< Cell1D propertoes, size 1 x NumberCell1D (marker)

    unsigned int NumberCell2D = 0; ///< number of Cell2D
    std::vector<unsigned int> Cell2DId = {}; ///< Cell2D id, size 1 x NumberCell2D
    std::vector<vector<unsigned int>> Cell2DVertices = {}; ///< Cell2D Vertices indices
    std::vector<vector<unsigned int>> Cell2DEdges = {}; ///< Cell2D Cell1D indices //prendi vector dalla libreria standar e fai un vettore di vettori
    std::vector<unsigned int> Cell2DMarkers = {};
};

PolygonalMesh transformChildrenFracturesToMesh(vector<Fracture>& fractures, double tol);

void saveMesh(PolygonalMesh& mesh, unsigned int idFracture);

}

#endif // POLYGONALMESH_HPP
