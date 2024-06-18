#pragma once

#include <Eigen/Eigen>
#include "GeometryLibrary.hpp"
#include "PolygonalMesh.hpp"

using namespace std;
using namespace GeometryLibrary;
using namespace PolygonalLibrary;

bool readFractures(const string& fileName, vector<Fracture>& fractures, const double& tol);

void printFractures(vector<Fracture> fractures, unsigned int expectedNumFractures);

void printTraces(vector<Trace> traces);

void printTracesToFile(const vector<Trace>& traces, const string& filename);

void printFracturesToFile(const vector<Fracture>& fractures, const string& filename);

void printFractureToDebug(const Fracture& fracture, const string& filename);

void printTraceToDebug(const Trace& trace, const string& filename);

void printPointToDebug(const Vector3d& point, const string& filename);

void saveMeshToFile(const PolygonalMesh& mesh, unsigned int idFracture);
