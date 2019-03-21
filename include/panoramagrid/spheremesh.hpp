#ifndef PANORAMAGRID_SPHEREMESH_HPP
#define PANORAMAGRID_SPHEREMESH_HPP


#include <panoramagrid/mesh.hpp>

namespace panoramagrid {

    class SphereMesh : public Mesh {
    public:
        SphereMesh(int stackCount, int sectorCount);

        std::vector<float> getVertices() override;

        std::vector<int> getIndices() override;

        DrawMethod getMethod() override;

    private:
        std::vector<float> vertices;
        std::vector<int> indices;

        void generateVertices(int stackCount, int sectorCount, float radius = 1.0f);

        void generateIndices(int stackCount, int sectorCount);
    };

}


#endif //PANORAMAGRID_SPHEREMESH_HPP
