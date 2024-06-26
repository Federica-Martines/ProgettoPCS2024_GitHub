#include "GeometryLibrary.hpp"
#include <iostream>
#include <Eigen/Eigen>
#include "PolygonalMesh.hpp"

using namespace std;
using namespace Eigen;
using namespace GeometryLibrary;
using namespace PolygonalLibrary;

namespace GeometryLibrary {


PolygonalMesh convertFractureToMesh(const Fracture& fracture, double tol) {
    PolygonalMesh mesh;
    vector<unsigned int> verticesId, edgesId;

    // Add vertices
    for (const Vector3d& vertex : fracture.vertices) {
        auto it = find_if(mesh.cells0D.begin(), mesh.cells0D.end(),
                          [&](const Cell0D& cell) { return areVectorsEqual(cell.coordinates, vertex, tol); });

        // se l'iteratore è end non l'ha trovato
        if (it == mesh.cells0D.end()) {
            Cell0D cell = Cell0D(mesh.NumberCell0D, vertex);
            mesh.cells0D.push_back(cell);
            verticesId.push_back(cell.id);
            mesh.NumberCell0D++;
        }
    }

    // Add edges
    for (unsigned int i = 0; i < mesh.NumberCell0D; i++) {

        // indices of the start and end vertices in cells0D
        unsigned int start = i;
        unsigned int end = (i + 1) %  mesh.NumberCell0D;

        Cell1D cell = Cell1D(mesh.NumberCell1D, start, end);
        cell.neighbours = {0};
        mesh.cells1D.push_back(cell);
        edgesId.push_back(cell.id);
        mesh.NumberCell1D++;
    }

    Vector3d normal = findNormal(fracture.vertices[0], fracture.vertices[1], fracture.vertices[2]);

    // Create Cell2D
    Cell2D cell2D = Cell2D(mesh.NumberCell2D, normal, verticesId, edgesId);
    mesh.NumberCell2D++;

    // Add Cell2D to the mesh
    mesh.cells2D.push_back(cell2D);

    return mesh;
}


bool areVectorsEqual(Vector3d v1, Vector3d v2, double tol) {
    return ((v2-v1).norm() < tol);
}

//questa funzione crea la sfera attorno alla frattura
BoundingSphere computeBoundingSphere(const std::vector<Vector3d>& vertices) {

    Vector3d centroid = Vector3d::Zero(); //inizializzo un centroide
    double maxRadius = 0.0;

    if(vertices.size() < 3) {
        cerr << "Computing Sphere with less than 3 points" << endl;
        return {centroid, maxRadius};
    }

    for (const auto& vertex : vertices) {
        centroid += vertex;
    }
    centroid /= vertices.size(); //il centroide è la mediana(?)

    for (const auto& vertex : vertices) {
        double distance = (vertex - centroid).norm();
        if (distance > maxRadius) {
            maxRadius = distance;
        }
    }

    return {centroid, maxRadius};
}

bool spheresIntersect(const BoundingSphere& sphere1, const BoundingSphere& sphere2) {
    double distance = (sphere1.centroid - sphere2.centroid).norm();
    double radiusSum = sphere1.radius + sphere2.radius;
    return distance <= radiusSum; //se è maggiore non si interesecano
}

// given 3 points not aligned returns the normal to the plane passing in the 3 points
Vector3d findNormal(const Vector3d p1, const Vector3d p2, const Vector3d p3) {

    // Calcolo i vettori che generano la normale
    Vector3d u1 = p3 - p1;
    Vector3d v1 = p1 - p2;
    return u1.cross(v1).normalized();
}

unsigned int findLyingPlane(const Vector3d n, double tol) {

    Vector3d eX = {1,0,0};
    Vector3d eY = {0,1,0};
    Vector3d eZ = {0,0,1};

    if (abs(n.dot(eX)) < tol && abs(n.dot(eY)) < tol) {
        return 1;
    }
    if (abs(n.dot(eX)) < tol && abs(n.dot(eZ)) < tol) {
        return 2;
    }
    if (abs(n.dot(eY)) < tol && abs(n.dot(eZ)) < tol) {
        return 3;
    }

    return 0;
}

// Function to project a 3D point onto the XY plane
Vector2d projectOntoXY(const Vector3d& point)
{
    return Vector2d(point.x(), point.y());
}

// Function to project a 3D point onto the XZ plane
Vector2d projectOntoXZ(const Vector3d& point)
{
    return Vector2d(point.x(), point.z());
}
// Function to project a 3D point onto the YZ plane
Vector2d projectOntoYZ(const Vector3d& point)
{
    return Vector2d(point.y(), point.z());
}

//proietta solo un punto, ovvero l'interesezione
void projectIntersection(Vector2d& projIntersection, Fracture F, Vector3d intersection) {

    switch (F.lyingPlane) {
    case 0:
    case 1:
        projIntersection = projectOntoXY(intersection);
        break;
    case 2:
        projIntersection = projectOntoXZ(intersection);
        break;
    case 3:
        projIntersection = projectOntoYZ(intersection);
        break;
    }

    return;
}

void projectVertices(vector<Vector2d>& projVertices, Fracture F) {

    switch (F.lyingPlane) { //switch è un if con i casi
    case 0:
    case 1:
        for (unsigned int ver = 0; ver < F.vertices.size(); ver++) {
            projVertices.push_back(projectOntoXY(F.vertices[ver]));
        }
        break;
    case 2:
        for (unsigned int ver = 0; ver < F.vertices.size(); ver++) {
            projVertices.push_back(projectOntoXZ(F.vertices[ver]));
        }
        break;
    case 3:
        for (unsigned int ver = 0; ver < F.vertices.size(); ver++) {
            projVertices.push_back(projectOntoYZ(F.vertices[ver]));
        }
        break;
    }

    return;
}


bool isPointOn2DSegment(const Vector2d& point, const Vector2d& s1, const Vector2d& s2, double tol) {
    Vector2d v1 = point - s1;
    Vector2d v2 = s2 - s1;
    double crossProduct = v1.x() * v2.y() - v1.y() * v2.x();
    if (fabs(crossProduct) > tol) return false; // Point is not collinear with the segment

    double dotProduct = v1.dot(v2);
    if (dotProduct < -tol || dotProduct > v2.squaredNorm() + tol) return false; // Point is not within the segment bounds

    return true;
}

bool isPointOn3DSegment(const Vector3d& point, const Vector3d& s1, const Vector3d& s2, double tol) {
    Vector3d v1 = point - s1;
    Vector3d v2 = s2 - s1;

    // Calculate cross product
    Vector3d crossProduct = v1.cross(v2);
    double crossProductNorm = crossProduct.norm();

    // Check if the cross product magnitude is greater than tolerance
    if (crossProductNorm > tol) return false; // Point is not collinear with the segment

    double dotProduct = v1.dot(v2);
    if (dotProduct < -tol || dotProduct > v2.squaredNorm() + tol) return false; // Point is not within the segment bounds

    return true;
}

// Ray casting algorithm (Point in polygon su wikipedia) prende una retta che passa dal punto e vede quante volte tale retta interseca il poligono
bool isPointIn2DPolygon(const Vector2d& point, const vector<Vector2d>& polygon, double tol) {
    int crossings = 0; //se il raggio passa atraverso il poligono
    int n = polygon.size();
    for (int i = 0; i < n; i++) {
        const Vector2d& p1 = polygon[i];
        const Vector2d& p2 = polygon[(i + 1) % n];

        if (isPointOn2DSegment(point, p1, p2, tol))
            return true; // Point lies on an edge

        if (fabs(p1.y() - p2.y()) < tol) // If the segment is horizontal, skip
            continue;

        if (point.y() < min(p1.y(), p2.y()) - tol) // If point is below the segment, skip
            continue;

        if (point.y() >= max(p1.y(), p2.y()) + tol) // If point is above the segment, skip
            continue;

        double x_intersect = (point.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();
        if (x_intersect > point.x() - tol) {
            if (p1.y() > p2.y()) crossings++;
            else crossings--;
        }
    }
    return crossings != 0;
}

int classifyTracePosition(const Vector3d& planePoint, const Vector3d& separatorPlane, const Vector3d& s1, const Vector3d& s2)
{
    // Compute the signed distances from the segment endpoints to the plane
    double d1 = separatorPlane.dot(s1 - planePoint);
    double d2 = separatorPlane.dot(s2 - planePoint);

    if (d1 > 0 && d2 > 0) {
        return 1; // Above the plane
    } else if (d1 < 0 && d2 < 0) {
        return -1; // Below the plane
    } else {
        return 0; // Crossing the plane
    }
}










bool existDirectionSegmentIntersection(Vector3d t1, Vector3d t2, Vector3d s1, Vector3d s2, double tol) {
    Vector3d cutDirection = t2 - t1;
    Vector3d segmentDirection = s1 - s2;

    // the lines are parallel
    if (cutDirection.cross(segmentDirection).norm() < tol) {
        return false;
    }

    // il segmento attraversa il piano
    Vector3d P =  s1 - t1;

    MatrixXd M(3, 2);

    M.col(0)=cutDirection;
    M.col(1)=segmentDirection;

    Vector2d solution = M.householderQr().solve(P);    //householderQr è in eigen
    double alpha = solution[0];
    double beta = solution[1];

    // the intersection is in outside the edge
    if (beta < -tol || beta > 1+tol || alpha < tol) {
        return false;
    }

    return true;
}

//t1, t2 estremi della traccia; s1, s2 estremi del lato (segmento)
int findLineSegmentIntersection(Vector3d& intersection,
                                const PolygonalMesh& mesh,
                                double& alpha,
                                double& beta,
                                const Trace cut,
                                const Cell1D edge,
                                double tol
                                ) {


    Vector3d start = mesh.cells0D[edge.start].coordinates;
    Vector3d end = mesh.cells0D[edge.end].coordinates;

    Vector3d cutDirection = cut.extremes[1] - cut.extremes[0];
    Vector3d segmentDirection = start - end;

    // the lines are parallel
    if (cutDirection.cross(segmentDirection).norm() < tol) {
        return -1;
    }

    // il segmento attraversa il piano
    Vector3d P =  start - cut.extremes[0];

    MatrixXd M(3, 2);

    M.col(0)=cutDirection;
    M.col(1)=segmentDirection;

    Vector2d solution = M.householderQr().solve(P);    //householderQr è in eigen
    alpha = solution[0];
    beta = solution[1];

    // the intersection is in outside the edge
    if (beta < -tol || beta > 1+tol) {
        return -1;
    }
    // the intersection is the vertex
    else if (abs(beta) <= tol || abs(beta-1) <= tol) {
        intersection = cut.extremes[0]+alpha*cutDirection;
        return 0;
    }

    // the intersection is inside the edge
    intersection = cut.extremes[0]+alpha*cutDirection;
    return 1;
}

}
