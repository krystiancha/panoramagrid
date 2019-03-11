#ifndef PANORAMAGRID_RENDERER_HPP
#define PANORAMAGRID_RENDERER_HPP


#include <memory>
#include <panoramagrid/node.hpp>
#include <panoramagrid/camera.hpp>

namespace panoramagrid {

    class Renderer {
    public:
        Renderer(int width, int height);

        virtual ~Renderer() = default;

        virtual void render(std::shared_ptr<Node> node) = 0;

        virtual void loadTexture(std::shared_ptr<Material> material) = 0;

        const std::shared_ptr<Camera> &getCamera() const;

        int getWidth() const;

        void setWidth(int width);

        int getHeight() const;

        void setHeight(int height);

    private:
        int width, height;
        std::shared_ptr<Camera> camera;
    };

}


#endif //PANORAMAGRID_RENDERER_HPP
