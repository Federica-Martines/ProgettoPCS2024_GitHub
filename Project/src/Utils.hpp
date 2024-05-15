#pragma once

#include <Eigen/Eigen>
#include <vector>
#include "GeometryLibrary.hpp"

using namespace std;
using namespace Geometry;

namespace Geometry {
unsigned int findTraces(vector<Trace>& traces, vector<Fracture> fractures, const double& tol);
}

namespace Utils {
unsigned int readFractures(const string& fileName, vector<Fracture>& fractures, const double& tol);

void printFractures(vector<Fracture> fractures, unsigned int expectedNumFractures);


void printTraces(vector<Trace> traces);
}

