#ifndef POLYGONALMESH_HPP
#define POLYGONALMESH_HPP
//PolygonalMesh.hpp

#pragma once

#include "Eigen/Eigen"


using namespace std;
using namespace Eigen;

namespace PolygonalLibrary {

struct Cell0D {
    unsigned int id;
    Vector3d coordinates = {};
    Cell0D() = default;
    Cell0D(unsigned int id, Vector3d coordinates) {
        this->id = id;
        this->coordinates = coordinates;
    }
    bool operator==(Cell0D cell){
        return id == cell.id;
    }
};

struct Cell1D {
    unsigned int id;
    Cell0D start;
    Cell0D end;
    vector<unsigned int> neighbours = {0}; // id delle celle2D adiacenti al lato
    Cell1D() = default;
    Cell1D(unsigned int id, Cell0D start, Cell0D end) {
        this->id = id;
        this->start = start;
        this->end = end;
    }

    bool operator==(Cell1D cell){
        return id == cell.id;
    }
};

struct Cell2D {
    unsigned int id;
    Vector3d normal;
    vector<Cell0D> vertices;
    vector<Cell1D> edges;
    vector<unsigned int> neighbours; // id delle celle2D adiacenti a sè stessa (anche solo per un vertice)
    Cell2D() = default;
    Cell2D(unsigned int id, Vector3d normal, vector<Cell0D> vertices, vector<Cell1D> edges) {
        this->id = id;
        this->normal = normal;
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
    Cell0D addCell0D(Vector3d coordinates) {
        Cell0D cell = Cell0D(NumberCell0D, coordinates);
        this->cells0D.push_back(cell);
        this->NumberCell0D++;
        return cell;
    }

    Cell1D addCell1D(Cell0D start, Cell0D end) {
        Cell1D cell = Cell1D(NumberCell1D, start, end);
        this->cells1D.push_back(cell);
        this->NumberCell1D++;
        return cell;
    }

    Cell2D addCell2D(Vector3d normal, vector<Cell0D> vertices, vector<Cell1D> edges) {
        Cell2D cell = Cell2D(NumberCell2D, normal, vertices, edges);
        this->cells2D.push_back(cell);
        this->NumberCell2D++;
        return cell;
    }
};

void saveMesh(const PolygonalMesh& mesh, unsigned int idFracture);

void splitEdge(PolygonalMesh& mesh, Cell2D cell, Cell1D edge, Cell0D newVertex);

bool pointInCell2D(const Vector3d& point, const Cell2D& cell, double tol);

bool findCellContainingPoint(Cell2D& foundCell, PolygonalMesh& mesh, Vector3d point, double tol);
}

#endif // POLYGONALMESH_HPP
