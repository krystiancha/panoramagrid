#ifndef PANORAMAGRID_MESH_HPP
#define PANORAMAGRID_MESH_HPP


#include <vector>

namespace panoramagrid {

    class Mesh {
    public:
        enum DrawMethod {
            TRIANGLE_STRIP,
        };

        Mesh();

        virtual std::vector<float> getVertices() = 0;

        virtual std::vector<int> getIndices() = 0;

        virtual DrawMethod getMethod() = 0;

        int getId() const;

    private:
        static int count;
        int id;
    };

}

#endif //PANORAMAGRID_MESH_HPP
