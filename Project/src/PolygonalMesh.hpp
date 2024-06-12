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

struct Cell0D {
    unsigned int id;
    Vector3d coordinates = {};
    Cell0D() = default;
    Cell0D(unsigned int id, Vector3d coordinates) {
        this->id = id;
        this->coordinates = coordinates;
    }
};

struct Cell1D {
    unsigned int id;
    unsigned int start;
    unsigned int end;
    vector<unsigned int> neighbours = {0}; // id delle celle2D adiacenti al lato
    Cell1D() = default;
    Cell1D(unsigned int id, unsigned int start, unsigned int end) {
        this->id = id;
        this->start = start;
        this->end = end;
    }
};

struct Cell2D {
    unsigned int id;
    vector<Cell0D> vertices;
    vector<Cell1D> edges;
    vector<unsigned int> neighbours; // id delle celle2D adiacenti a sè stessa (anche solo per un vertice)
    Cell2D() = default;
    Cell2D(unsigned int id, vector<Cell0D> vertices, vector<Cell1D> edges) {
        this->id = id;
        this->vertices = vertices;
        this->edges = edges;
    }
};

struct PolygonalMesh
{
    unsigned int NumberCell0D = 0; ///< number of Cell0D
    vector<Cell0D> cells0D;

    unsigned int NumberCell1D = 0; ///< number of Cell1D
    vector<Cell1D> cells1D;

    unsigned int NumberCell2D = 0; ///< number of Cell2D
    vector<Cell2D> cells2D;

    // crea e aggiunge un vertice
    unsigned int addCell0D(Vector3d coordinates) {
        Cell0D cell = Cell0D(NumberCell0D, coordinates);
        this->cells0D.push_back(cell);
        this->NumberCell0D++;
        return cell.id;
    }

    void addCell1D(unsigned int start, unsigned int end) {
        Cell1D cell = Cell1D(NumberCell1D, start, end);
        this->cells1D.push_back(cell);
        this->NumberCell1D++;
    }
};



PolygonalMesh convertFractureToMesh(const Fracture& fracture, double tol);

PolygonalMesh transformFractureToMesh(vector<Fracture>& fractures, double tol);

void saveMesh(const PolygonalMesh& mesh, unsigned int idFracture);


}

#endif // POLYGONALMESH_HPP
