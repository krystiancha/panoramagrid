#ifndef PANORAMAGRID_NODE_HPP
#define PANORAMAGRID_NODE_HPP


#include <memory>
#include <array>
#include <panoramagrid/mesh.hpp>
#include <panoramagrid/material.hpp>

namespace panoramagrid {

    class Node {
    public:
        Node(
            const std::shared_ptr<Mesh> &mesh, const std::shared_ptr<Material> &material,
            const std::array<float, 3> &position = {0, 0, 0},
            const std::array<float, 3> &orientation = {0, 0, 0}, // rpy
            const std::array<float, 3> &scale = {1, 1, 1}
        );

        const std::shared_ptr<Mesh> &getMesh() const;

        void setMesh(const std::shared_ptr<Mesh> &mesh);

        const std::shared_ptr<Material> &getMaterial() const;

        void setMaterial(const std::shared_ptr<Material> &material);

        const std::array<float, 3> &getPosition() const;

        void setPosition(const std::array<float, 3> &position);

        const std::array<float, 3> &getOrientation() const;

        void setOrientation(const std::array<float, 3> &orientation);

        const std::array<float, 3> &getScale() const;

        void setScale(const std::array<float, 3> &scale);

    private:
        std::shared_ptr<Mesh> mesh;
        std::shared_ptr<Material> material;
        std::array<float, 3> position;
        std::array<float, 3> orientation;
        std::array<float, 3> scale;
    };

}


#endif //PANORAMAGRID_NODE_HPP
