#include <vector>
#include <iostream>
#include "PolygonalMesh.hpp"
#include "GeometryLibrary.hpp"

using namespace PolygonalLibrary;
using namespace GeometryLibrary;

namespace PolygonalLibrary {

void splitEdge(unsigned int& newVertex, Cell2D& cell2D, PolygonalMesh& mesh, Cell1D& edge, Vector3d intersection){
    newVertex = mesh.addCell0D(intersection);

    unsigned int leftEdgeId = mesh.addCell1D(cell2D.id, edge.start, newVertex);
    unsigned int rightEdgeId = mesh.addCell1D(cell2D.id, newVertex, edge.end);


    for (unsigned int n = 0; n < edge.neighbours.size(); n++) {
        unsigned int neighbourId = edge.neighbours[n];
        Cell2D& cellToUpdate = mesh.cells2D[neighbourId];

        for (unsigned int e = 0; e < cellToUpdate.edges.size(); e++) {
            if (cellToUpdate.edges[e] == edge.id) {

                cellToUpdate.vertices.insert(cellToUpdate.vertices.begin() + e+1, newVertex);

                // bool found = false;
                // for (unsigned int i = 0; i < cellToUpdate.vertices.size(); i++) {
                //     if (edge.start == cellToUpdate.vertices[i])
                //     {
                //         found = true;
                //         break;
                //     }
                // }

                if(n == 0){
                    cellToUpdate.edges[e] = leftEdgeId;
                    cellToUpdate.edges.insert(cellToUpdate.edges.begin() + e+1, rightEdgeId);
                }
                else {
                    cellToUpdate.edges[e] = rightEdgeId;
                    cellToUpdate.edges.insert(cellToUpdate.edges.begin() + e+1, leftEdgeId);
                }
                break;
            }
        }
    }


    mesh.cells1D[edge.id].alive = false;

}


unsigned int findNeighbour(const PolygonalMesh& mesh, unsigned int cellId, unsigned int edgeId) {
    Cell1D edge = mesh.cells1D[edgeId];

    for (unsigned int& n : edge.neighbours){
        if (n != cellId){
            return n;
        }
    }
    cerr << "Nessun vicino trovato per il lato: " << edgeId << endl;
    return NULL;
}

void generateCell2D(PolygonalMesh& mesh, const unsigned int& cell2DId, const unsigned int& intersectionId, const unsigned int& intersectionNextId) {

    bool writeLeft = true;
    vector<unsigned int> leftCell2DVertices, rightCell2DVertices, leftCell2DEdges, rightCell2DEdges;
    int intCounter = 0;
    const Cell2D cell2D = mesh.cells2D[cell2DId];

    for (unsigned int nVertex = 0; nVertex < cell2D.vertices.size(); nVertex++) {
        unsigned int v = cell2D.vertices[nVertex];
        unsigned int newEdgeId = cell2D.edges[nVertex];

        if ((v == intersectionId) || (v == intersectionNextId)) {
            intCounter++;

            // aggiungiamo i vertici
            leftCell2DVertices.push_back(v);
            rightCell2DVertices.push_back(v);

            /* aggiungiamo il lato del taglio alle nuove celle 2D */
            if (intCounter == 2) {
                unsigned int cutEdgeId = mesh.addCell1D(cell2D.id, intersectionId, intersectionNextId);
                leftCell2DEdges.push_back(cutEdgeId);
                rightCell2DEdges.push_back(cutEdgeId);
            }

            /* aggiungiamo i lati */
            if (nVertex == 0){
                if (writeLeft)
                    leftCell2DEdges.push_back(newEdgeId);
                else
                    rightCell2DEdges.push_back(newEdgeId);
            }
            else {
                if (writeLeft)
                    rightCell2DEdges.push_back(newEdgeId);
                else
                    leftCell2DEdges.push_back(newEdgeId);
            }

            writeLeft = !writeLeft;
        }
        else {
            if (writeLeft) {
                leftCell2DVertices.push_back(v);
                leftCell2DEdges.push_back(newEdgeId);
            }
            else{
                rightCell2DVertices.push_back(v);
                rightCell2DEdges.push_back(newEdgeId);

            }
        }
    }

    unsigned int newLeftCellId = mesh.addCell2D(cell2D.normal, leftCell2DVertices, leftCell2DEdges);
    unsigned int newRightCellId = mesh.addCell2D(cell2D.normal, rightCell2DVertices, rightCell2DEdges);

    Cell2D newLeftCell = mesh.cells2D[newLeftCellId];
    Cell2D newRightCell = mesh.cells2D[newRightCellId];

    // aggiorno i vicini dei lati creati
    for (unsigned int& newCellEdgeId : newLeftCell.edges) {
        Cell1D& newCellEdge = mesh.cells1D[newCellEdgeId];
        if (newCellEdge.neighbours[0] == cell2D.id) {newCellEdge.neighbours[0] = newLeftCell.id; continue;}
        if (newCellEdge.neighbours[1] == cell2D.id) {newCellEdge.neighbours[1] = newLeftCell.id; continue;}
        newCellEdge.neighbours.push_back(newLeftCell.id);
    }
    for (unsigned int& newCellEdgeId : newRightCell.edges) {
        Cell1D& newCellEdge = mesh.cells1D[newCellEdgeId];
        if (newCellEdge.neighbours[0] == cell2D.id) {newCellEdge.neighbours[0] = newRightCell.id; continue;}
        if (newCellEdge.neighbours[1] == cell2D.id) {newCellEdge.neighbours[1] = newRightCell.id; continue;}
        newCellEdge.neighbours.push_back(newRightCell.id);
    }

    // spendo la cella vecchia
    mesh.cells2D[cell2DId].alive = false;
}







bool pointInCell2D(const PolygonalMesh& mesh, const Vector3d& rayOrigin, const Cell2D& cell, double tol) {
    const auto& vertices = cell.vertices;

    Vector3d rayEnd = rayOrigin + mesh.cells0D[vertices[1]].coordinates - mesh.cells0D[vertices[0]].coordinates; // Direction of the ray (can be any non-parallel direction)

    int intersectionCount = 0;
    size_t numVertices = vertices.size();

    // Iterate over polygon edges
    for (unsigned int i = 0; i < numVertices;  i++) {
        unsigned int idxI = vertices[i];
        unsigned int idxJ = vertices[(i+1) % numVertices];
        const Vector3d& vi = mesh.cells0D[idxI].coordinates;
        const Vector3d& vj = mesh.cells0D[idxJ].coordinates;


        if (isPointOn3DSegment(rayOrigin, vi, vj, tol)) {
            return true;
        }

        // Check if the ray intersects the edge vi-vj
        if (existDirectionSegmentIntersection(rayOrigin, rayEnd, vi, vj, tol)){
            intersectionCount++;
        }

    }



    // If intersection count is odd, point is inside the polygon
    return (intersectionCount % 2 == 1);
}

bool findCellContainingPoint(Cell2D& foundCell, PolygonalMesh& mesh, Vector3d point, double tol) {
    for (Cell2D& cell : mesh.cells2D) {
        if(cell.alive) {
            if (pointInCell2D(mesh, point, cell, tol))  {
                foundCell = cell;
                return true;
            }
        }
    }
    return false;
}

}

























