#include <panoramagrid/renderer.hpp>

namespace panoramagrid {

    Renderer::Renderer(int width, int height)
        : width(width), height(height), camera(std::make_shared<Camera>(1.0f * width / height)) {}

    const std::shared_ptr<Camera> &Renderer::getCamera() const {
        return camera;
    }

    int Renderer::getWidth() const {
        return width;
    }

    void Renderer::setWidth(int width) {
        Renderer::width = width;
        camera->setAspectRatio(1.0f * width / height);
    }

    int Renderer::getHeight() const {
        return height;
    }

    void Renderer::setHeight(int height) {
        Renderer::height = height;
        camera->setAspectRatio(1.0f * width / height);
    }

}
