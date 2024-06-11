#include <vector>
#include <fstream>
#include <iostream>
#include "PolygonalMesh.hpp"
#include "GeometryLibrary.hpp"
#include <filesystem>

    using namespace GeometryLibrary;
using namespace PolygonalLibrary;
namespace fs = std::filesystem;


namespace PolygonalLibrary {

PolygonalMesh transformChildrenFracturesToMesh(vector<Fracture>& fractures, double tol) {
    PolygonalMesh mesh;

    unsigned int vertexId = 0;
    unsigned int edgeId = 0;
    unsigned int FracId = 0;

    for (const auto& F : fractures) {
        vector<unsigned int> currentPolygonVertices;

        // Add vertices
        for (const Vector3d& v : F.vertices) {

            bool found = false;
            unsigned int foundVectorId;
            for (unsigned int compV = 0; compV < mesh.Cell0DCoordinates.size(); compV++) {
                if (areVectorsEqual(v, mesh.Cell0DCoordinates[compV], tol)) {
                    found = true;
                    foundVectorId = compV;
                    break;
                }
            }

            if (!found){
                mesh.Cell0DId.push_back(vertexId);
                mesh.Cell0DCoordinates.push_back(v);
                vertexId++;
                currentPolygonVertices.push_back(vertexId);
            }
            else {
                currentPolygonVertices.push_back(foundVectorId);
            }

        }

        // Add edges
        vector<unsigned int> currentPolygonEdges;
        for (unsigned int i = 0; i < F.numVertices; i++) {
            int from = currentPolygonVertices[i];
            int to = currentPolygonVertices[(i + 1) % F.numVertices];

            bool found = false;
            unsigned int foundEdgeId;
            for (unsigned int e = 0; e < mesh.Cell1DVertices.size(); e++) {
                Vector2i compE = mesh.Cell1DVertices[e];
                if ((compE[0] == from && compE[1] == to ) || (compE[0] == to && compE[1] == from)) {
                    found = true;
                    foundEdgeId = e;
                    break;
                }
            }

            Vector2i edge = Vector2i{static_cast<int>(from), static_cast<int>(to)};
            if (!found) {
                mesh.Cell1DId.push_back(edgeId);
                mesh.Cell1DVertices.push_back(edge);
                edgeId++;
            }
            else {
                currentPolygonEdges.push_back(foundEdgeId);
            }
        }

        // Add polygons
        mesh.Cell2DId.push_back(FracId++);
        mesh.Cell2DVertices.push_back(currentPolygonVertices); // salva il vettore di ID vei vertici di questa frattura
        mesh.Cell2DEdges.push_back(currentPolygonEdges);
    }

    mesh.NumberCell0D = vertexId;
    mesh.NumberCell1D = edgeId;
    mesh.NumberCell2D = fractures.size();

    return mesh;
}


void saveMesh(PolygonalMesh& mesh, unsigned int idFracture) {
    // Create folder structure
    std::string folderName = "polygonalMeshes/mesh" + std::to_string(idFracture);
    string fileNameCell0D = folderName + "/Cell0D.txt";
    string fileNameCell1D = folderName + "/Cell1D.txt";
    string fileNameCell2D = folderName + "/Cell2D.txt";

    // Create folder
    try {
        fs::create_directories(folderName);
    } catch (const std::exception& e) {
        std::cerr << "Error creating folder: " << e.what() << std::endl;
        return;
    }


    // Save Cell0D data
    ofstream cell0DFile(fileNameCell0D);
    cell0DFile << "Id;X;Y;Z" << endl;
    for (unsigned int i = 0; i < mesh.NumberCell0D; i++) {
        cell0DFile << mesh.Cell0DId[i] << "; "
                   << mesh.Cell0DCoordinates[i].x() << "; "
                   << mesh.Cell0DCoordinates[i].y() << "; "
                   << mesh.Cell0DCoordinates[i].z()
                   << endl;
    }
    cell0DFile.close();

    // Save Cell1D data
    ofstream cell1DFile(fileNameCell1D);
    cell1DFile << "Id;Origin;End" << endl;
    for (unsigned int i = 0; i < mesh.NumberCell1D; i++) {
        cell1DFile << mesh.Cell1DId[i] << ";" << mesh.Cell1DVertices[i][0] << ";"
                   << mesh.Cell1DVertices[i][1] << endl;
    }
    cell1DFile.close();
    // Save Cell2D data
    ofstream cell2DFile(fileNameCell2D);
    cell2DFile << "Id;NumVertices;Vertices;NumEdges;Edges" << endl;
    for (unsigned int i = 0; i < mesh.NumberCell2D; i++) {
        cell2DFile << mesh.Cell2DId[i] << ";" << mesh.Cell2DVertices[i].size() << ";";
        for (const auto& vertexId : mesh.Cell2DVertices[i]) {
            cell2DFile << vertexId << ";";
        }
        cell2DFile << mesh.Cell2DEdges[i].size() << ";";
        for (const auto& edgeId : mesh.Cell2DEdges[i]) {
            cell2DFile << edgeId << ";";
        }
        cell2DFile << endl;
    }
    cell2DFile.close();

    cout << "Mesh data saved successfully." << endl;
}

}
