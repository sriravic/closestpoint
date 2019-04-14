#ifndef __MESH_H__
#define __MESH_H__

#pragma once
#include <common.h>
#include <geometry.h>
#include <matrix.h>
#include <string>
#include <vector>

/// Supports loading triangular obj file meshes.
template<typename DataType, typename IndexType>
class Mesh : public NonCopyable
{
public:
    using VectorType        = Vector3<DataType>;
    using IndexVectorType   = Vector3<IndexType>;
    using BoxType           = Box3<DataType>;
    using SphereType        = Sphere<DataType>;
    using TriangleType      = Triangle<DataType>;
    using IndexBufferType   = std::vector<IndexVectorType>;
    using DataBufferType    = std::vector<VectorType>;

    bool open(const std::string& filename);
    
    BoxType getBounds() const noexcept
    {
        return myBounds;
    }

    SphereType getBoundingSphere() const
    {
        return ::getBoundingSphere<DataType>(myBounds);
    }

    IndexType getNumPrimitives() const noexcept
    {
        return static_cast<IndexType>(myIndexBuffer.size());
    }

    const IndexBufferType& getIndexBuffer() const noexcept
    { 
        return myIndexBuffer;
    }
    
    const DataBufferType& getVertexBuffer() const noexcept
    {
        return myVertexBuffer;
    }

    TriangleType getPrimitive(const IndexType idx) const
    {
        assert(idx < myIndexBuffer.size());
        IndexVectorType vidx = myIndexBuffer[idx];
        return { myVertexBuffer[vidx[0]],
                 myVertexBuffer[vidx[1]],
                 myVertexBuffer[vidx[2]] };
    }

    BoxType getBounds(IndexType idx) const
    {
        assert(idx < myIndexBuffer.size());
        BoxType ret;
        IndexVectorType vidx = myIndexBuffer[idx];
        for (int i = 0; i < 3; i++)
            ret.grow(myVertexBuffer[vidx[i]]);
        return ret;
    }

    void printStats() const noexcept;

private:
    IndexBufferType myIndexBuffer;
    DataBufferType  myVertexBuffer;
    Box3<DataType>  myBounds;
};

using MeshF = Mesh<float, uint32_t>;
using MeshD = Mesh<double, uint32_t>;

#endif

