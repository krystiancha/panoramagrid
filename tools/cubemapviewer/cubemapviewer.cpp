/*!
 * The cubemapviewer application opens a cubemap image passed as an execution argument and enables the user to look
 * around like a photo sphere.
 *
 * Controls:
 * Mouse -- look around
 * Scroll wheel -- field of view (zoom)
 * ESC -- free the mouse pointer, quit the application if pressed the second time
 * Left click -- enable looking around again after pressing ESC
 * q -- quit
 */

#include <iostream>
#include <boost/algorithm/clamp.hpp>
#include <opencv2/opencv.hpp>
#include <panoramagrid/gl/glapplication.hpp>
#include <panoramagrid/cubemesh.hpp>
#include <panoramagrid/cubemapmaterial.hpp>

namespace pg = panoramagrid;

class CubemapViewer : public pg::gl::applications::GlApplication {
public:
    void parseArgs(int argc, char **argv) override {
        boost::program_options::options_description opt("Allowed options");
        opt.add_options()("input,i", boost::program_options::value<std::string>()->required());
        options.add(opt);

        GlApplication::parseArgs(argc, argv);

        try {
            inputFile = vm["input"].as<std::string>();
        } catch (boost::bad_any_cast &e) {
            throw std::runtime_error("Bad input filename");
        }
    }

    void initContext() override {
        GlApplication::initContext();
        nodes.push_back(node);

        glfwGetCursorPos(window, &lastCursorX, &lastCursorY);
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwSwapInterval(1);
    }

    void run() override {
        loadTexture();

        glEnable(GL_DEPTH_TEST);

        while (!glfwWindowShouldClose(window)) {
            if (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED) {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                for (const auto &node : nodes) {
                    renderer->render(node);
                }
                glfwSwapBuffers(window);
            }

            glfwWaitEvents();
        }
    }

    void framebufferSizeCallback(int width, int height) override {
        GlApplication::framebufferSizeCallback(width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        getRenderer()->render(node);
        glfwSwapBuffers(window);
    }

    void keyCallback(int key, int scancode, int action, int mods) override {
        GlApplication::keyCallback(key, scancode, action, mods);
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            if (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED) {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            } else {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
        }
        if (key == GLFW_KEY_Q && action == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
    }

    void cursorPosCallback(double xpos, double ypos) override {
        GlApplication::cursorPosCallback(xpos, ypos);

        if (glfwGetInputMode(window, GLFW_CURSOR) != GLFW_CURSOR_DISABLED) {
            return;
        }

        auto rpy = renderer->getCamera()->getOrientation();

        renderer->getCamera()->setOrientation({
                                                      0,
                                                      boost::algorithm::clamp<float>(
                                                              static_cast<float>(rpy[1] + (ypos - lastCursorY) *
                                                                                          cursorSensitivity),
                                                              -M_PI_2f32 + std::numeric_limits<float>::epsilon(),
                                                              M_PI_2f32 - std::numeric_limits<float>::epsilon()
                                                      ),
                                                      static_cast<float>(rpy[2] -
                                                                         (xpos - lastCursorX) * cursorSensitivity)
                                              });

        lastCursorX = xpos;
        lastCursorY = ypos;
    }

    void mouseButtonCallback(int button, int action, int mods) override {
        GlApplication::mouseButtonCallback(button, action, mods);

        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

            glfwGetCursorPos(window, &lastCursorX, &lastCursorY);
        }
    }

    void scrollCallback(double xoffset, double yoffset) override {
        GlApplication::scrollCallback(xoffset, yoffset);

        if (glfwGetInputMode(window, GLFW_CURSOR) != GLFW_CURSOR_DISABLED) {
            return;
        }

        renderer->getCamera()->setFov(
                boost::algorithm::clamp<float>(
                        static_cast<float>(
                                renderer->getCamera()->getFov() - yoffset * scrollSensitivity
                        ),
                        std::numeric_limits<float>::epsilon(),
                        M_PIf32 - std::numeric_limits<float>::epsilon()
                )
        );
    }

protected:
    double lastCursorX = 0, lastCursorY = 0;
    double cursorSensitivity = 0.002, scrollSensitivity = 0.05;
    std::string inputFile = "";
    std::shared_ptr<pg::Node> node = std::make_shared<pg::Node>(std::make_shared<pg::CubeMesh>(),
                                                                std::make_shared<pg::CubemapMaterial>());

    void loadTexture() {
        node->getMaterial()->setTexture(cv::imread(inputFile));
        getRenderer()->render(node);
        getRenderer()->loadTexture(node->getMaterial());
    }
};

int main(int argc, char *argv[]) {
    CubemapViewer app;
    app.parseArgs(argc, argv);

    CubemapViewer::initGlfw();
    app.initContext();

    app.run();

    return 0;
}
