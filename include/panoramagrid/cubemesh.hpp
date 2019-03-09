#ifndef PANORAMAGRID_CUBEMESH_HPP
#define PANORAMAGRID_CUBEMESH_HPP


#include <panoramagrid/mesh.hpp>

namespace panoramagrid {

    class CubeMesh : public Mesh {
    public:
        std::vector<float> getVertices() override;

        std::vector<int> getIndices() override;

        DrawMethod getMethod() override;
    };

}


#endif //PANORAMAGRID_CUBEMESH_HPP
