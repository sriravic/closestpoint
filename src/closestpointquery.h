#ifndef __CLOSEST_POINT_QUERY_H__
#define __CLOSEST_POINT_QUERY_H__

#pragma once
#include <accel.h>
#include <common.h>
#include <mesh.h>
#include <queryfunctions.h>
#include <string>

enum class QueryMethod
{
    QUERY_BRUTE_FORCE,
    QUERY_OPTIMIZED
};

/// The query manager class is responsible for handling all nearest
/// neighbour point queries. Initialize with a mesh and method
/// before running any queries
template<typename DataType, typename IndexType>
class ClosestPointQuery : public NonCopyable
{
public:
    using VectorType        = Vector3<DataType>;
    using AccelType         = KdTree<DataType, IndexType>;
    using MeshType          = Mesh<DataType, IndexType>;
    using QueryResultType   = QueryResult<DataType, IndexType>;
    using InputType         = std::vector<VectorType>;
    using OutputType        = std::vector<QueryResultType>;
    
    /// Init the query structure with a mesh file to be used for querying
    /// returns true if the loading of the mesh passed
    bool    init(const std::string& meshname, QueryMethod m);

    /// Multiple points with a max search radius
    void    query(const InputType& pts, QueryMethod m, OutputType& results,
               DataType searchRadius = std::numeric_limits<DataType>::max()) const noexcept;

    const MeshType& getMesh() const { return myMesh; }

private:
    
    void    bruteforce(const InputType& pts, OutputType& results,
                DataType searchRadius) const noexcept;

    void    queryKdTree(const InputType& pts, OutputType& results,
                DataType searchRadius) const noexcept;

    MeshType    myMesh;
    AccelType   myAccel;
    bool        myInitialized = false;
};

using ClosestPointQueryF = ClosestPointQuery<float, uint32_t>;
using ClosestPointQueryD = ClosestPointQuery<double, uint32_t>;

#endif