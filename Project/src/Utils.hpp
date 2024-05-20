#pragma once

#include <Eigen/Eigen>
#include <vector>
#include "GeometryLibrary.hpp"

using namespace std;
using namespace Geometry;

unsigned int findTraces(vector<Trace>& traces, vector<Fracture>& fractures, const double& tol);

bool checkSegmentIntersection(vector<Vector3d>& intersections, const Vector3d planeNormal, Vector3d planePoint, Vector3d a, Vector3d b, double tol);

void findIntersections(Trace& trace, Fracture F1, Fracture F2, double tol);

void sortTraces(vector<Fracture>& fractures);

