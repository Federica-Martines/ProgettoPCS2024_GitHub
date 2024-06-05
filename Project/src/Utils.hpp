#pragma once

#include <Eigen/Eigen>
#include <vector>
#include "GeometryLibrary.hpp"
#include <deque>

using namespace std;
using namespace GeometryLibrary;

unsigned int findTraces(vector<Trace>& traces, vector<Fracture>& fractures, const double& tol);

bool checkSegmentPlaneIntersection(vector<Vector3d>& intersections, const Vector3d planeNormal, Vector3d planePoint, Vector3d a, Vector3d b, double tol);

void findIntersections(Trace& trace, Fracture F1, Fracture F2, double tol);

void sortTraces(vector<Fracture>& fractures);

void cuttingFracture(vector<Fracture>& resultFractures, Fracture& frac, deque<Trace>& cuts, double tol);

bool checkTraceTips(Fracture F, Trace T, double tol);

void addTraceToFractures(Fracture& F1, Fracture& F2, Trace& trace, double tol);

void splitFracture(vector<Fracture>& subFractures, vector<Vector3d>& cutPoints, const Fracture& F, const Vector3d& t1, const Vector3d& t2, double tol);
