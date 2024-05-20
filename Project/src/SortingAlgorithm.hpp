#define __SORTING_ALGORITHM_H

#include <vector>
#include <functional> // for std::function
#include "GeometryLibrary.hpp"
using namespace Geometry;

namespace SortLibrary {


void Merge(std::vector<Trace>& v, unsigned int sx, unsigned int cx, unsigned int dx, std::function<bool(const Trace&, const Trace&, double)> compare, double tol) {
    unsigned int i = sx;
    unsigned int j = cx + 1;

    std::vector<Trace> b;
    b.reserve(dx - sx + 1);

    while (i <= cx && j <= dx) {
        if (compare(v[i], v[j], tol))
            b.push_back(v[i++]);
        else
            b.push_back(v[j++]);
    }

    while (i <= cx)
        b.push_back(v[i++]);
    while (j <= dx)
        b.push_back(v[j++]);

    for (unsigned int k = sx; k <= dx; ++k)
        v[k] = b[k - sx];
}


void MergeSort(std::vector<Trace>& v, unsigned int sx, unsigned int dx, std::function<bool(const Trace&, const Trace&, double)> compare, double tol) {
    if (sx < dx) {
        unsigned int cx = (sx + dx) / 2;
        MergeSort(v, sx, cx, compare, tol);
        MergeSort(v, cx + 1, dx, compare, tol);

        Merge(v, sx, cx, dx, compare, tol);
    }
}

void DoMergeSort(std::vector<Trace>& v, std::function<bool(const Trace&, const Trace&, double)> compare, double tol) {
    MergeSort(v, 0, v.size() - 1, compare, tol);
}

} // namespace SortLibrary


