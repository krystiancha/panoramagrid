#include <iostream>

#include <panoramagrid/panoramagrid.hpp>
#include <panoramagrid/gl/gl.hpp>

#include <GLFW/glfw3.h>

namespace pg = panoramagrid;

class SimpleApplication {
public:

    SimpleApplication(pg::Renderer &renderer) : renderer(renderer) {}

    void initialize() {
        std::cerr << "GLFW " << glfwGetVersionString() << std::endl;
        glfwSetErrorCallback([](int error, const char *description) {
            throw std::runtime_error(std::string("GLFW Error: ") + description);
        });
        glfwInit();
    }

    void createWindow() {
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
        glfwWindowHint(GLFW_FOCUSED, GLFW_FALSE);
        glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
        window = glfwCreateWindow(640, 480, "Simple", nullptr, nullptr);

        glfwSetWindowUserPointer(window, this);
        glfwSetFramebufferSizeCallback(window, [](GLFWwindow* window, int width, int height) {
            getThis(window)->framebufferSizeCallback(window, width, height);
        });
    }

    void configureContext() {
        glfwMakeContextCurrent(window);
        gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);
    }

    void configureInput() {
        glfwSetKeyCallback(window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
            getThis(window)->keyCallback(window, key, scancode, action, mods);
        });
        glfwSetCursorPosCallback(window, [](GLFWwindow* window, double xpos, double ypos) {
            getThis(window)->cursorPositionCallback(window, xpos, ypos);
        });
        glfwSetMouseButtonCallback(window, [](GLFWwindow* window, int button, int action, int mods) {
            getThis(window)->mouseButtonCallback(window, button, action, mods);
        });
        glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
            getThis(window)->scrollCallback(window, xoffset, yoffset);
        });
        glfwGetCursorPos(window, &lastXpos, &lastYpos);
    }

    void destroyWindow() {
        glfwDestroyWindow(window);
    }

    void terminate() {
        glfwTerminate();
    }

    void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
        glViewport(0, 0, width, height);
        renderer.setWidth(width);
        renderer.setHeight(height);
    }

    void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            if (glfwGetInputMode(window, GLFW_CURSOR) != GLFW_CURSOR_DISABLED) {
                glfwSetWindowShouldClose(window, GL_TRUE);
            } else {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
        }
    }

    void cursorPositionCallback(GLFWwindow* window, double xpos, double ypos) {
        if (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED) {
            renderer.getCamera()->setRelativeOrientation({
                glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) ? static_cast<float>(xpos - lastXpos) * -0.01f : 0,
                !glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) ? static_cast<float>(xpos - lastXpos) * -0.01f : 0,
                static_cast<float>(ypos - lastYpos) * -0.01f,
            });
        }

        lastXpos = xpos;
        lastYpos = ypos;
    }

    void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        }
    }

    void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
        renderer.getCamera()->setFov(renderer.getCamera()->getFov() + yoffset * 0.1f);
    }

    GLFWwindow *getWindow() const {
        return window;
    }

    bool isKeyPressed(int key) {
        return glfwGetKey(window, key) == GLFW_PRESS;
    }

private:
    GLFWwindow *window;
    pg::Renderer &renderer;
    double lastXpos, lastYpos;

    static SimpleApplication *getThis(GLFWwindow* window) {
        return static_cast<SimpleApplication *>(glfwGetWindowUserPointer(window));
    }
};

int main(int argc, char *argv[]) {
    pg::gl::GlRenderer renderer(1280, 1080);

    SimpleApplication application(renderer);
    application.initialize();
    application.createWindow();
    application.configureContext();
    application.configureInput();

    std::vector<std::shared_ptr<pg::Node>> nodes {
        std::make_shared<pg::Node>(
            std::make_shared<pg::SphereMesh>(18, 36),
            std::make_shared<pg::UvMaterial>()
        ),
        std::make_shared<pg::Node>(
                std::make_shared<pg::SphereMesh>(18, 36),
                std::make_shared<pg::UvMaterial>(),
                std::array<float, 3> {2, 0, 0}
        ),
    };
    nodes[0]->getMaterial()->setTexture(cv::imread("/home/krystiancha/.local/share/panoramagrid/sample_equirectangular.jpg"));
    renderer.render(nodes[0]);
    renderer.loadTexture(nodes[0]->getMaterial());

    nodes[1]->getMaterial()->setTexture(cv::imread("/home/krystiancha/.local/share/panoramagrid/sample_equirectangular.jpg"));
    renderer.render(nodes[1]);
    renderer.loadTexture(nodes[1]->getMaterial());

    glfwSwapInterval(1);
    glEnable(GL_DEPTH_TEST);
    while (!glfwWindowShouldClose(application.getWindow())) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (glfwGetInputMode(application.getWindow(), GLFW_CURSOR) == GLFW_CURSOR_DISABLED) {
            renderer.getCamera()->setRelativePosition({
                application.isKeyPressed(GLFW_KEY_A) * 0.01f - application.isKeyPressed(GLFW_KEY_D) * 0.01f,
                application.isKeyPressed(GLFW_KEY_LEFT_SHIFT) * 0.01f - application.isKeyPressed(GLFW_KEY_SPACE) * 0.01f,
                application.isKeyPressed(GLFW_KEY_S) * 0.01f - application.isKeyPressed(GLFW_KEY_W) * 0.01f,
            });
        }

        renderer.render(nodes[0]);
        renderer.render(nodes[1]);
        glfwSwapBuffers(application.getWindow());
        glfwPollEvents();
    }

    application.destroyWindow();
    application.terminate();
}