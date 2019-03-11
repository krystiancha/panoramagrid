#ifndef PANORAMAGRID_CUBEMAPVIEWERGLAPPLICATION_HPP
#define PANORAMAGRID_CUBEMAPVIEWERGLAPPLICATION_HPP


#include <panoramagrid/gl/applications/glapplication.hpp>
#include <panoramagrid/cubemesh.hpp>
#include <panoramagrid/cubemapmaterial.hpp>

namespace panoramagrid::gl::applications {

    class CubemapViewer : public GlApplication {
    public:
        void parseArgs(int argc, char **argv) override;

        void initContext() override;

        void loadCubemap(std::string filename);

        void run() override;

        void framebufferSizeCallback(int width, int height) override;

        void keyCallback(int key, int scancode, int action, int mods) override;

        void cursorPosCallback(double xpos, double ypos) override;

        void mouseButtonCallback(int button, int action, int mods) override;

        void scrollCallback(double xoffset, double yoffset) override;

    protected:
        double lastCursorX = 0, lastCursorY = 0;
        double cursorSensitivity = 0.002, scrollSensitivity = 0.05;

        std::shared_ptr<Node> cubeNode = std::make_shared<Node>(std::make_shared<CubeMesh>(),
            std::make_shared<CubemapMaterial>());
    };

}


#endif //PANORAMAGRID_CUBEMAPVIEWERGLAPPLICATION_HPP
