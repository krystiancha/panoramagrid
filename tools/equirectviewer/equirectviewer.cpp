#include <boost/algorithm/clamp.hpp>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <opencv2/opencv.hpp>
#include <panoramagrid/gl/glapplication.hpp>
#include <panoramagrid/spheremesh.hpp>
#include <panoramagrid/uvmaterial.hpp>

namespace pg = panoramagrid;

class EquirectViewer : public pg::gl::applications::GlApplication {
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

        if (rightHeld) {
            r = static_cast<float>(r - (xpos - lastCursorX) * cursorSensitivity);
        } else {
            p = boost::algorithm::clamp<float>(
                    static_cast<float>(p - (ypos - lastCursorY) * cursorSensitivity),
                    -M_PI_2f32,
                    M_PI_2f32
            );
            y = static_cast<float>(y + (xpos - lastCursorX) * cursorSensitivity);
        }

        glm::mat4 matp(1), maty(1), matr(1);
        matp = glm::rotate(matp, p, glm::vec3(1, 0, 0));
        maty = glm::rotate(maty, y, glm::vec3(0, -1, 0));
        matr = glm::rotate(matr, r, glm::vec3(0, 0, -1));
        glm::quat quat = glm::quat_cast(matr * matp * maty);
        renderer->getCamera()->setOrientation({quat[0], quat[1], quat[2], quat[3]});

        lastCursorX = xpos;
        lastCursorY = ypos;
    }

    void mouseButtonCallback(int button, int action, int mods) override {
        GlApplication::mouseButtonCallback(button, action, mods);

        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

            glfwGetCursorPos(window, &lastCursorX, &lastCursorY);
        } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            rightHeld = action == GLFW_PRESS;
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
    bool rightHeld = false;
    float r = 0, p = 0, y = 0;
    std::string inputFile = "";
    std::shared_ptr<pg::Node> node = std::make_shared<pg::Node>(std::make_shared<pg::SphereMesh>(18, 36),
                                                        std::make_shared<pg::UvMaterial>());

    void loadTexture() {
        node->getMaterial()->setTexture(cv::imread(inputFile));
        getRenderer()->render(node);
        getRenderer()->loadTexture(node->getMaterial());
    }
};

int main(int argc, char *argv[]) {
    EquirectViewer app;
    app.parseArgs(argc, argv);

    EquirectViewer::initGlfw();
    app.initContext();

    app.run();

    return 0;
}
