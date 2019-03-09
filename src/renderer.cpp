#include <panoramagrid/renderer.hpp>

namespace panoramagrid {

    Renderer::Renderer(int width, int height, const std::shared_ptr<Camera> &camera)
        : width(width), height(height), camera(camera) {}

    const std::shared_ptr<Camera> &Renderer::getCamera() const {
        return camera;
    }

    int Renderer::getWidth() const {
        return width;
    }

    int Renderer::getHeight() const {
        return height;
    }

}
