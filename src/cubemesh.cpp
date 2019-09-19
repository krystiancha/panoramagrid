#include <panoramagrid/cubemesh.hpp>

namespace panoramagrid {

    std::vector<float> CubeMesh::getVertices() {
        return {
                -1, -1, -1, 0, 0,
                +1, -1, -1, 0, 0,
                -1, -1, +1, 0, 0,
                +1, -1, +1, 0, 0,
                -1, +1, -1, 0, 0,
                +1, +1, -1, 0, 0,
                +1, +1, +1, 0, 0,
                -1, +1, +1, 0, 0,
        };
    }

    std::vector<int> CubeMesh::getIndices() {
        return {3, 2, 6, 7, 4, 2, 0, 3, 1, 6, 5, 4, 1, 0};
    }

    Mesh::DrawMethod CubeMesh::getMethod() {
        return TRIANGLE_STRIP;
    }

}

