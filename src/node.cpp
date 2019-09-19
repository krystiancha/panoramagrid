#include <panoramagrid/node.hpp>

namespace panoramagrid {

    Node::Node(const std::shared_ptr<Mesh> &mesh, const std::shared_ptr<Material> &material,
               const std::array<float, 3> &position, const std::array<float, 3> &orientation,
               const std::array<float, 3> &scale) : mesh(mesh), material(material), position(position),
                                                    orientation(orientation), scale(scale) {}

    const std::shared_ptr<Mesh> &Node::getMesh() const {
        return mesh;
    }

    void Node::setMesh(const std::shared_ptr<Mesh> &mesh) {
        Node::mesh = mesh;
    }

    const std::shared_ptr<Material> &Node::getMaterial() const {
        return material;
    }

    void Node::setMaterial(const std::shared_ptr<Material> &material) {
        Node::material = material;
    }

    const std::array<float, 3> &Node::getPosition() const {
        return position;
    }

    void Node::setPosition(const std::array<float, 3> &position) {
        Node::position = position;
    }

    const std::array<float, 3> &Node::getOrientation() const {
        return orientation;
    }

    void Node::setOrientation(const std::array<float, 3> &orientation) {
        Node::orientation = orientation;
    }

    const std::array<float, 3> &Node::getScale() const {
        return scale;
    }

    void Node::setScale(const std::array<float, 3> &scale) {
        Node::scale = scale;
    }

}
