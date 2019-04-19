#include <panoramagrid/gl/applications/glapplication.hpp>
#include <panoramagrid/gl/glrenderer.hpp>
#include <iostream>

namespace panoramagrid::gl::applications {

    void GlApplication::parseArgs(int argc, char **argv) {
        boost::program_options::options_description opt("Allowed options");
        opt.add_options()
            ("help,h", "Print help")
            ("width", boost::program_options::value<int>()->default_value(1280))
            ("height", boost::program_options::value<int>()->default_value(720))
            ("fullscreen,f", "View in fullscreen");

        options.add(opt);

        boost::program_options::store(
            boost::program_options::command_line_parser(argc, argv)
                .options(options)
                .positional(positionalOptions)
                .run(),
            vm);

        if (vm.count("help")) {
            std::cout << opt << std::endl;
            exit(0);
        }

        renderer = std::make_shared<GlRenderer>(
            vm["width"].as<int>(),
            vm["height"].as<int>());
    }

    void GlApplication::initGlfw() {
        glfwInit();
        glfwSetErrorCallback([](int error, const char *description) {
            throw std::logic_error((std::string) "GLFW Error: " + description);
        });
    }

    void GlApplication::initContext() {
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        window = glfwCreateWindow(
            renderer->getWidth(),
            renderer->getHeight(),
            "Cubemap Viewer",
            vm.count("fullscreen") ? glfwGetPrimaryMonitor() : nullptr,
            nullptr
        );
        glfwSetWindowUserPointer(window, this);
        glfwMakeContextCurrent(window);
        gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);
        initCallbacks();
    }

    const std::shared_ptr<Renderer> &GlApplication::getRenderer() const {
        return renderer;
    }

    void GlApplication::framebufferSizeCallback(int width, int height) {
        glViewport(0, 0, width, height);
        renderer->setHeight(height);
        renderer->setWidth(width);
    }

    void GlApplication::initCallbacks() {
        glfwSetFramebufferSizeCallback(window, [](GLFWwindow *window, int width, int height) {
            auto that = static_cast<GlApplication *>(glfwGetWindowUserPointer(window));
            that->framebufferSizeCallback(width, height);
        });
        glfwSetKeyCallback(window, [](GLFWwindow *window, int key, int scancode, int action, int mods) {
            auto that = static_cast<GlApplication *>(glfwGetWindowUserPointer(window));
            that->keyCallback(key, scancode, action, mods);
        });
        glfwSetCursorPosCallback(window, [](GLFWwindow *window, double xpos, double ypos) {
            auto that = static_cast<GlApplication *>(glfwGetWindowUserPointer(window));
            that->cursorPosCallback(xpos, ypos);
        });
        glfwSetMouseButtonCallback(window, [](GLFWwindow *window, int button, int action, int mods) {
            auto that = static_cast<GlApplication *>(glfwGetWindowUserPointer(window));
            that->mouseButtonCallback(button, action, mods);
        });
        glfwSetScrollCallback(window, [](GLFWwindow *window, double xoffset, double yoffset) {
            auto that = static_cast<GlApplication *>(glfwGetWindowUserPointer(window));
            that->scrollCallback(xoffset, yoffset);
        });
    }

}
