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
    Cell0D(unsigned int id, Vector3d coordinates):id(id), coordinates(coordinates) { }
    bool operator==(Cell0D cell){
        return id == cell.id;
    }
};

struct Cell1D {
    unsigned int id;
    unsigned int start;
    unsigned int end;
    vector<unsigned int> neighbours = {0}; // id delle celle2D adiacenti al lato
    Cell1D() = default;
    Cell1D(unsigned int id, unsigned int start, unsigned int end): id(id), start(start), end(end) {}

    bool operator==(Cell1D cell){
        return id == cell.id;
    }
};

struct Cell2D {
    unsigned int id;
    Vector3d normal;
    vector<unsigned int> vertices;
    vector<unsigned int> edges;
    bool alive = true; // id delle celle2D adiacenti a sè stessa (anche solo per un vertice)
    Cell2D() = default;
    Cell2D(unsigned int id,
           const Vector3d& normal,
           const vector<unsigned int>& vertices,
           const vector<unsigned int>& edges):
        id(id),
        normal(normal),
        vertices(vertices),
        edges(edges)
    {}
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
        cells0D.push_back(cell);
        NumberCell0D++;
        return cell.id;
    }

    unsigned int addCell1D(unsigned int start, unsigned int end) {
        Cell1D cell = Cell1D(NumberCell1D, start, end);
        cells1D.push_back(cell);
        NumberCell1D++;
        return cell.id;
    }

    unsigned int addCell2D(Vector3d normal,
                     const vector<unsigned int>& vertices,
                     const vector<unsigned int>& edges) {
        Cell2D cell = Cell2D(NumberCell2D, normal, vertices, edges);
        cells2D.push_back(cell);
        NumberCell2D++;
        return cell.id;
    }
};

void saveMesh(const PolygonalMesh& mesh, unsigned int idFracture);

void splitEdge(vector<unsigned int> splitEdges, unsigned int& newVertex, PolygonalMesh& mesh, Cell1D edge, Vector3d intersection);

void updateNeighbours(unsigned int oldEdge, vector<unsigned int> splitEdges, PolygonalMesh& mesh, unsigned int cell1, unsigned int cell2);

unsigned int findNeighbour(const PolygonalMesh& mesh, unsigned int cellId, unsigned int edgeId);

void generateCell2D(PolygonalMesh& mesh, Cell2D& cell2D, unsigned int& intersectionId, unsigned int& intersectionNextId );

bool pointInCell2D(const PolygonalMesh& mesh, const Vector3d& point, const Cell2D& cell, double tol);

bool findCellContainingPoint(Cell2D& foundCell, PolygonalMesh& mesh, Vector3d point, double tol);
}

#endif // POLYGONALMESH_HPP
