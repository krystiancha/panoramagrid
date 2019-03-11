#include <panoramagrid/gl/applications/cubemapviewer.hpp>
#include <boost/algorithm/clamp.hpp>
#include <opencv2/opencv.hpp>

void panoramagrid::gl::applications::CubemapViewer::framebufferSizeCallback(int width, int height) {
    GlApplication::framebufferSizeCallback(width, height);
    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    getRenderer()->render(cubeNode);
    glfwSwapBuffers(window);
    renderer->setHeight(height);
    renderer->setWidth(width);
}

void
panoramagrid::gl::applications::CubemapViewer::keyCallback(int key, int scancode, int action, int mods) {
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

void panoramagrid::gl::applications::CubemapViewer::cursorPosCallback(double xpos, double ypos) {
    GlApplication::cursorPosCallback(xpos, ypos);

    if (glfwGetInputMode(window, GLFW_CURSOR) != GLFW_CURSOR_DISABLED) {
        return;
    }

    auto rpy = renderer->getCamera()->getOrientation();

    renderer->getCamera()->setOrientation({
        0,
        boost::algorithm::clamp<float>(
            static_cast<float>(rpy[1] - (ypos - lastCursorY) * cursorSensitivity),
            -M_PI_2f32 + std::numeric_limits<float>::epsilon(),
            M_PI_2f32 - std::numeric_limits<float>::epsilon()
        ),
        static_cast<float>(rpy[2] - (xpos - lastCursorX) * cursorSensitivity)
    });

    lastCursorX = xpos;
    lastCursorY = ypos;
}

void panoramagrid::gl::applications::CubemapViewer::mouseButtonCallback(int button, int action, int mods) {
    GlApplication::mouseButtonCallback(button, action, mods);

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        glfwGetCursorPos(window, &lastCursorX, &lastCursorY);
    }
}

void panoramagrid::gl::applications::CubemapViewer::scrollCallback(double xoffset, double yoffset) {
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

void panoramagrid::gl::applications::CubemapViewer::loadCubemap(std::string filename) {

    cubeNode->getMaterial()->setTexture(cv::imread(filename));
    getRenderer()->render(cubeNode);
    getRenderer()->loadTexture(cubeNode->getMaterial());
}

void panoramagrid::gl::applications::CubemapViewer::initContext() {
    GlApplication::initContext();
    nodes.push_back(cubeNode);

    glfwGetCursorPos(window, &lastCursorX, &lastCursorY);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSwapInterval(1);
}

void panoramagrid::gl::applications::CubemapViewer::run() {
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

void panoramagrid::gl::applications::CubemapViewer::parseArgs(int argc, char **argv) {
    boost::program_options::options_description opt("Allowed options");
    opt.add_options()("input", boost::program_options::value<std::string>()->required());

    options.add(opt);

    GlApplication::parseArgs(argc, argv);
}
