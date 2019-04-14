#ifndef __QUERY_FUNCTIONS_H__
#define __QUERY_FUNCTIONS_H__

#pragma once
#include <common.h>
#include <geometry.h>
#include <unordered_map>
#include <ostream>

/// This file contains all the nearest neighbour query functions
/// between points and geometric primitives

/// For a query point, there is a potential chance 
/// that many points might be 'considered' same distance
/// (machine precision). Hence we store all such points
/// and the ids of the triangles that contain them
template<typename DataType, typename IndexType>
struct QueryResult
{
    using VectorType = Vector3<DataType>;
    using HashMap    = std::unordered_map<IndexType, VectorType>;

    explicit QueryResult(DataType closest = std::numeric_limits<DataType>::max())
        : myClosestDistance(closest)
    {
        // revert to default in case we're initialized with 
        // crazy max2 value
        if (!isValid(myClosestDistance))
            myClosestDistance = std::numeric_limits<DataType>::max();
    }

    // convenient function to clear existsing results
    void clear()
    {
        myClosestDistance = std::numeric_limits<DataType>::max();
        myClosestPoints.clear();
    }

    bool operator == (const QueryResult& R) const noexcept
    {
        return myClosestDistance == R.myClosestDistance;
    }

    bool operator <= (const QueryResult& R) const noexcept
    {
        return myClosestDistance <= R.myClosestDistance;
    }

    bool operator < (const QueryResult& R) const noexcept
    {
        return myClosestDistance < R.myClosestDistance;
    }

    bool operator > (const QueryResult& R) const noexcept
    {
        return myClosestDistance > R.myClosestDistance;
    }

    // TODO: add some clearer name
    void appendResults(const QueryResult& R)
    {
        assert(isValid(R.myClosestDistance));
        assert(R.myClosestDistance <= myClosestDistance);
        for (auto&& pt : R.myClosestPoints)
            myClosestPoints[pt.first] = pt.second;
        myClosestDistance = R.myClosestDistance;
    }

    friend std::ostream& operator<< (std::ostream& out, const QueryResult& R)
    {
        out << std::endl << " { " << std::endl;
        out << "\t Dist2 : " << R.myClosestDistance << std::endl;
        for (auto&& c : R.myClosestPoints)
        {
            out << " \t Id    : " << c.first << " "
                << " Point : " << c.second << std::endl;
        }
        out << " } " << std::endl;
        return out;
    }

    // We use a hashmap because for a single query point
    // multiple points in the mesh might be the smallest distance
    HashMap  myClosestPoints;
    DataType myClosestDistance;
};

template<
    typename DataType,
    typename VectorType    = Vector3<DataType>,
    typename BoxType       = Box3<DataType>>
static DataType
squaredClosestDistance(const BoxType& box, const VectorType& querypt)
{
    DataType res = DataType(0);
    for (int i = 0; i < 3; i++)
    {
        if (querypt[i] < box.bmin[i])
            res += (querypt[i] - box.bmin[i]) * (querypt[i] - box.bmin[i]);
        if (querypt[i] > box.bmax[i])
            res += (querypt[i] - box.bmax[i]) * (querypt[i] - box.bmax[i]);
    }
    return res;
}

/// Computes closest squared distance
template<
    typename DataType,
    typename IndexType,
    typename VectorType         = Vector3<DataType>,
    typename BoxType            = Box3<DataType>,
    typename QueryResultType    = QueryResult<DataType, IndexType>>
static void
computeBoxClosestPoint(const BoxType& box, const VectorType& input,
    QueryResultType& output)
{
    VectorType res = input;
    res = VectorType::max(res, box.bmin);
    res = VectorType::min(res, box.bmin);
    
    output.myClosestDistance = (res - input).length2();
    output.myClosestPoints[0] = res;    // using a dummy id of '0'
}

/// Returns closest point and squared distance
template<
    typename DataType,
    typename IndexType,
    typename VectorType         = Vector3<DataType>,
    typename TriangleType       = Triangle<DataType>,
    typename QueryResultType    = QueryResult<DataType, IndexType>>
static void
computeTriangleClosestPoint(const TriangleType& triangle,
    const VectorType& input, const IndexType idx, QueryResultType& output)
{
    // Check if input in vertex region outside v0
    VectorType ab = triangle.v[1] - triangle.v[0];
    VectorType ac = triangle.v[2] - triangle.v[0];
    VectorType ap = input         - triangle.v[0];

    DataType d1 = ab.dot(ap);
    DataType d2 = ac.dot(ap);
    if (d1 <= DataType(0) && d2 <= DataType(0))
    {
        // barycentric coordinates (1, 0, 0);
        auto&& res = triangle.v[0];
        output.myClosestDistance = (res - input).length2();
        output.myClosestPoints[idx] = res;
        return;
    }

    // check if input is in vertex region outside v1
    VectorType bp = input - triangle.v[1];
    DataType d3 = ab.dot(bp);
    DataType d4 = ac.dot(bp);
    if (d3 >= DataType(0) && d4 <= d3)
    {
        auto&& res = triangle.v[1];
        output.myClosestDistance = (res - input).length2();
        output.myClosestPoints[idx] = res;
        return;
    }

    // check if input is in edge region of AB
    // return projection in case
    DataType vc = d1 * d4 - d3 * d2;
    if (vc <= DataType(0) && 
        d1 >= DataType(0) &&
        d3 <= DataType(0))
    {
        DataType v = d1 / (d1 - d3);
        auto&& res = triangle.v[0] + v * ab;
        output.myClosestDistance = (res - input).length2();
        output.myClosestPoints[idx] = res;
        return;
    }

    // cehck if input is vertex region outside C
    VectorType cp = input - triangle.v[2];
    DataType d5 = ab.dot(cp);
    DataType d6 = ac.dot(cp);
    if (d6 >= DataType(0) && d5 <= d6)
    {
        auto&& res = triangle.v[2];
        output.myClosestDistance = (res - input).length2();
        output.myClosestPoints[idx] = res;
        return;
    }

    // check if P is in edge region of AC
    DataType vb = d5 * d2 - d1 * d6;
    if (vb <= DataType(0) &&
        d2 >= DataType(0) &&
        d6 <= DataType(0))
    {
        DataType w = d2 / (d2 - d6);
        auto&& res = triangle.v[0] + w * ac;
        output.myClosestDistance = (res - input).length2();
        output.myClosestPoints[idx] = res;
        return;
    }

    // check if P is in edge region of  BC
    // return projection onto edge BC
    DataType va = d3 * d6 - d5 * d4;
    if (va <= DataType(0) &&
        (d4 - d3) >= DataType(0) &&
        (d5 - d6) >= DataType(0))
    {
        DataType w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        auto&& res = triangle.v[1] + w * (triangle.v[2] - triangle.v[1]);
        output.myClosestDistance = (res - input).length2();
        output.myClosestPoints[idx] = res;
        return;
    }

    // Point projects to interior of triangle
    DataType denom = DataType(1) / (va + vb + vc);
    DataType v = vb * denom;
    DataType w = vc * denom;
    auto&& res = triangle.v[0] + ab * v + ac * w;
    output.myClosestDistance = (res - input).length2();
    output.myClosestPoints[idx] = res;
}

#endif
