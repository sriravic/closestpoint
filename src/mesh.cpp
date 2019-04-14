#include <mesh.h>
#include <iostream>
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#include <util.h>

template<typename DataType, typename IndexType>
bool
Mesh<DataType, IndexType>::open(const std::string& file)
{
    tinyobj::attrib_t               attribs;
    std::vector<tinyobj::shape_t>   shapes;
    std::string                     msg;
    Timer                           timer("Mesh Loader");
    timer.start();
    std::cout << "Loading : " << file << std::endl;
    bool ret = tinyobj::LoadObj(&attribs, &shapes, nullptr, &msg, file.c_str());
    if (ret)
    {
        
        size_t numVertices = attribs.vertices.size() / 3;
        assert(attribs.vertices.size() % 3 == 0);

        // Vertex Data
        myVertexBuffer.resize(numVertices);
        for (size_t vtx = 0; vtx < numVertices; vtx++)
        {
            VectorType v(
                attribs.vertices[3 * vtx + 0],
                attribs.vertices[3 * vtx + 1],
                attribs.vertices[3 * vtx + 2]);
            myVertexBuffer[vtx] = v;
            myBounds.grow(v);
        }

        // Index Data
        for (auto&& shape : shapes)
        {
            size_t index_offset = 0;
            for (size_t face = 0, nfaces = shape.mesh.num_face_vertices.size();
                face < nfaces; face++)
            {
                // only triangles supported
                assert(shape.mesh.num_face_vertices[face] == 3);
                const auto& indices = shape.mesh.indices;
                myIndexBuffer.emplace_back(
                    static_cast<IndexType>(indices[index_offset + 0].vertex_index),
                    static_cast<IndexType>(indices[index_offset + 1].vertex_index),
                    static_cast<IndexType>(indices[index_offset + 2].vertex_index));
                index_offset += 3;
            }
        }
        ret = true;
    }
    else
        ret = false;

    timer.stop();
    std::cout << "Loading : " << file << " [ "
              << (ret ? "Passed" : "Failed") << " ]" << std::endl;
    timer.printElapsed();
    if (!ret)
        std::cout << "Error Status : " << msg << std::endl;
    return ret;
}

template<typename DataType, typename IndexType>
void
Mesh<DataType, IndexType>::printStats() const noexcept
{
    std::cout << "Mesh Stats" << std::endl;
    std::cout << "----------" << std::endl;
    std::cout << "Mesh Triangles : " << myIndexBuffer.size() << std::endl;
    std::cout << "Mesh Bounds    : " << myBounds << std::endl;
    std::cout << "-----------------" << std::endl;
}


// explicit template instatiations
template class Mesh<float, uint32_t>;
template class Mesh<double, uint32_t>;
