#pragma once

#include <Eigen/Eigen>
#include <vector>
#include "GeometryLibrary.hpp"

using namespace std;

namespace Geometry {
unsigned int readFractures(const string& fileName, vector<Fracture>& vec, const double& tol);
}

