#include <panoramagrid/mesh.hpp>

namespace panoramagrid {

    Mesh::Mesh() {
        id = count++;
    }

    int Mesh::getId() const {
        return id;
    }

    int Mesh::count = 0;

}
