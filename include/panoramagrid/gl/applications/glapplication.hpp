#ifndef PANORAMAGRID_APPLICATION_HPP
#define PANORAMAGRID_APPLICATION_HPP


#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <panoramagrid/renderer.hpp>
#include <boost/program_options.hpp>

namespace panoramagrid::gl::applications {

    class GlApplication {
    public:
        virtual void parseArgs(int argc, char **argv);

        static void initGlfw();

        virtual void initContext();

        virtual void run() = 0;

        const std::shared_ptr<Renderer> &getRenderer() const;

        virtual void framebufferSizeCallback(int width, int height) {};

        virtual void keyCallback(int key, int scancode, int action, int mods) {};

        virtual void cursorPosCallback(double xpos, double ypos) {};

        virtual void mouseButtonCallback(int button, int action, int mods) {};

        virtual void scrollCallback(double xoffset, double yoffset) {};

    protected:
        boost::program_options::options_description options;
        boost::program_options::positional_options_description positionalOptions;
        std::vector<std::shared_ptr<Node>> nodes;
        GLFWwindow *window;
        std::shared_ptr<Renderer> renderer;
        boost::program_options::variables_map vm;

    private:
        void initCallbacks();
    };

}


#endif //PANORAMAGRID_APPLICATION_HPP
