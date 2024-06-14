#include <vector>
#include <fstream>
#include <iostream>
#include "PolygonalMesh.hpp"
#include "GeometryLibrary.hpp"
#include <filesystem>


using namespace PolygonalLibrary;
using namespace GeometryLibrary;
namespace fs = filesystem;


namespace PolygonalLibrary {

void splitEdge(PolygonalMesh& mesh, Cell2D cell, Cell1D edge, unsigned int newVertex){
    unsigned int leftEdgeId = mesh.addCell1D(edge.start, newVertex);
    unsigned int rightEdgeId = mesh.addCell1D(newVertex, edge.end);

    mesh.cells1D[leftEdgeId].neighbours = edge.neighbours;
    mesh.cells1D[rightEdgeId].neighbours = edge.neighbours;

    // aggiorno i lati della cella vicina
    auto it = find(edge.neighbours.begin(), edge.neighbours.end(), cell.id);
    Cell2D& cellToUpdate = mesh.cells2D[*it];

    for (unsigned int e = 0; e < cellToUpdate.edges.size(); e++) {
        if (cellToUpdate.edges[e] == edge.id) {
            cellToUpdate.edges.erase(cellToUpdate.edges.begin() + e);
            cellToUpdate.edges.push_back(leftEdgeId);
            cellToUpdate.edges.push_back(rightEdgeId);
            break;
        }
    }

}

void saveMesh(const PolygonalMesh& mesh, unsigned int idFracture) {
    // Create folder structure
    string folderName = "polygonalMeshes/mesh" + to_string(idFracture);
    string fileNameCell0D = folderName + "/Cell0D.txt";
    string fileNameCell1D = folderName + "/Cell1D.txt";
    string fileNameCell2D = folderName + "/Cell2D.txt";

    // Create folder
    try {
        fs::create_directories(folderName);
    } catch (const exception& e) {
        cerr << "Error creating folder: " << e.what() << endl;
        return;
    }

    // Save Cell0D data
    ofstream cell0DFile(fileNameCell0D);
    cell0DFile << "Id;X;Y;Z" << endl;
    for (const auto& cell : mesh.cells0D) {
        cell0DFile << cell.id << "; " << setprecision(16) << scientific
                   << cell.coordinates.x() << "; "
                   << cell.coordinates.y() << "; "
                   << cell.coordinates.z()
                   << endl;
    }
    cell0DFile.close();

    // Save Cell1D data
    ofstream cell1DFile(fileNameCell1D);
    cell1DFile << "Id;Origin;End" << endl;
    for (const auto& cell : mesh.cells1D) {
        cell1DFile << cell.id << ";" << cell.start << ";" << cell.end << endl;
    }
    cell1DFile.close();

    // Save Cell2D data
    ofstream cell2DFile(fileNameCell2D);
    cell2DFile << "Id;NumVertices;Vertices;NumEdges;Edges" << endl;
    for (const auto& cell : mesh.cells2D) {
        cell2DFile << cell.id << ";" << cell.vertices.size() << ";";
        for (const auto& vertex : cell.vertices) {
            cell2DFile << vertex << ";";
        }
        cell2DFile << cell.edges.size() << ";";
        for (const auto& edge : cell.edges) {
            cell2DFile << edge << ";";
        }
        cell2DFile << endl;
    }
    cell2DFile.close();

    cout << "Mesh data saved successfully." << endl;
}

bool pointInCell2D(const PolygonalMesh& mesh, const Vector3d& rayOrigin, const Cell2D& cell, double tol) {
    const auto& vertices = cell.vertices;


    Vector3d rayEnd = rayOrigin + mesh.cells0D[1].coordinates - mesh.cells0D[0].coordinates; // Direction of the ray (can be any non-parallel direction)

    int intersectionCount = 0;
    size_t numVertices = vertices.size();

    // Iterate over polygon edges
    for (unsigned int i = 0; i < numVertices;  i++) {
        const Vector3d& vi = mesh.cells0D[i].coordinates;
        const Vector3d& vj = mesh.cells0D[(i+1) % numVertices].coordinates;

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
        if (pointInCell2D(mesh, point, cell, tol))  {
            foundCell = cell;
            return true;
        }
    }
    return false;
}

}

























