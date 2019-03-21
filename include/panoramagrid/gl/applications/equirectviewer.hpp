#ifndef PANORAMAGRID_EQUIRECTVIEWER_HPP
#define PANORAMAGRID_EQUIRECTVIEWER_HPP


#include <string>
#include <panoramagrid/node.hpp>
#include <panoramagrid/panoramagrid.hpp>
#include <panoramagrid/spheremesh.hpp>
#include <panoramagrid/uvmaterial.hpp>
#include <panoramagrid/gl/applications/glapplication.hpp>

namespace panoramagrid::gl::applications {

    class EquirectViewer : public GlApplication {
    public:
        void parseArgs(int argc, char **argv) override;

        void initContext() override;

        void run() override;

        void framebufferSizeCallback(int width, int height) override;

        void keyCallback(int key, int scancode, int action, int mods) override;

        void cursorPosCallback(double xpos, double ypos) override;

        void mouseButtonCallback(int button, int action, int mods) override;

        void scrollCallback(double xoffset, double yoffset) override;

    protected:
        double lastCursorX = 0, lastCursorY = 0;
        double cursorSensitivity = 0.002, scrollSensitivity = 0.05;
        std::string inputFile = "";
        std::shared_ptr<Node> node = std::make_shared<Node>(std::make_shared<SphereMesh>(36, 72),
            std::make_shared<UvMaterial>());

        void loadTexture();
    };

}


#endif //PANORAMAGRID_EQUIRECTVIEWER_HPP
