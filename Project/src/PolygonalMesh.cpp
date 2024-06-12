#include <vector>
#include <fstream>
#include <iostream>
#include "PolygonalMesh.hpp"
#include "GeometryLibrary.hpp"
#include <filesystem>

    using namespace GeometryLibrary;
using namespace PolygonalLibrary;
namespace fs = filesystem;


namespace PolygonalLibrary {

PolygonalMesh convertFractureToMesh(const Fracture& fracture, double tol) {
    PolygonalMesh mesh;

    // Add vertices
    for (const Vector3d& vertex : fracture.vertices) {
        auto it = find_if(mesh.cells0D.begin(), mesh.cells0D.end(),
                          [&](const Cell0D& cell) { return areVectorsEqual(cell.coordinates, vertex, tol); });

        // se l'iteratore è end non l'ha trovato
        if (it == mesh.cells0D.end()) {
            Cell0D cell0D = Cell0D(mesh.NumberCell0D, vertex);
            mesh.cells0D.push_back(cell0D);
            mesh.NumberCell0D++;
        }
    }

    // Add edges
    for (unsigned int i = 0; i < mesh.NumberCell0D; i++) {

        // indices of the start and end vertices in cells0D
        unsigned int start = mesh.cells0D[i].id;
        unsigned int end = mesh.cells0D[(i + 1) %  mesh.NumberCell0D].id;

        Cell1D cell1D = Cell1D(mesh.NumberCell1D, start, end);

        mesh.cells1D.push_back(cell1D);
        mesh.NumberCell1D++;
    }

    // Create Cell2D
    Cell2D cell2D = Cell2D(mesh.NumberCell2D, mesh.cells0D, mesh.cells1D);

    // Add Cell2D to the mesh
    mesh.cells2D.push_back(cell2D);

    return mesh;
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
            cell2DFile << vertex.id << ";";
        }
        cell2DFile << cell.edges.size() << ";";
        for (const auto& edge : cell.edges) {
            cell2DFile << edge.id << ";";
        }
        cell2DFile << endl;
    }
    cell2DFile.close();

    cout << "Mesh data saved successfully." << endl;
}


}
