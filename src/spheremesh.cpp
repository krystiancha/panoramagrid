#include <panoramagrid/spheremesh.hpp>
#include <cmath>

namespace panoramagrid {

    SphereMesh::SphereMesh(int stackCount, int sectorCount) {
        generateVertices(stackCount, sectorCount, 1);
        generateIndices(stackCount, sectorCount);
    }

    std::vector<float> SphereMesh::getVertices() {
        return vertices;
    }

    std::vector<int> SphereMesh::getIndices() {
        return indices;
    }

    Mesh::DrawMethod SphereMesh::getMethod() {
        return TRIANGLES;
    }

    void SphereMesh::generateVertices(int stackCount, int sectorCount, float radius) {
        float stackStep = M_PIf32 / stackCount;
        float sectorStep = 2 * M_PIf32 / sectorCount;

        for (int i = 0; i <= stackCount; ++i) {
            float stackAngle = M_PI_2f32 - i * stackStep;
            float xz = radius * cosf(stackAngle);
            float y = radius * sinf(stackAngle);

            for (int j = 0; j <= sectorCount; ++j) {
                float sectorAngle = M_PIf32 + j * sectorStep;

                float x = xz * sinf(sectorAngle);
                float z = xz * cosf(sectorAngle);

                vertices.push_back(x);
                vertices.push_back(y);
                vertices.push_back(z);

                vertices.push_back(static_cast<float>(j) / sectorCount);
                vertices.push_back(static_cast<float>(i) / stackCount);
            }
        }
    }

    void SphereMesh::generateIndices(int stackCount, int sectorCount) {
        for (int i = 0; i < stackCount; ++i) {
            int k1 = i * (sectorCount + 1);
            int k2 = k1 + sectorCount + 1;

            for (int j = 0; j < sectorCount; ++j) {
                if (i != 0) {
                    indices.push_back(k1);
                    indices.push_back(k2);
                    indices.push_back(k1 + 1);
                }

                if (i != (stackCount - 1)) {
                    indices.push_back(k1 + 1);
                    indices.push_back(k2);
                    indices.push_back(k2 + 1);
                }

                ++k1;
                ++k2;
            }
        }
    }
}