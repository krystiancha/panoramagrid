#include <panoramagrid/gl/applications/equirectviewer.hpp>
#include <boost/algorithm/clamp.hpp>
#include <glm/ext.hpp>

void panoramagrid::gl::applications::EquirectViewer::framebufferSizeCallback(int width, int height) {
    GlApplication::framebufferSizeCallback(width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    getRenderer()->render(node);
    glfwSwapBuffers(window);
}

void
panoramagrid::gl::applications::EquirectViewer::keyCallback(int key, int scancode, int action, int mods) {
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

void panoramagrid::gl::applications::EquirectViewer::cursorPosCallback(double xpos, double ypos) {
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

void panoramagrid::gl::applications::EquirectViewer::mouseButtonCallback(int button, int action, int mods) {
    GlApplication::mouseButtonCallback(button, action, mods);

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        glfwGetCursorPos(window, &lastCursorX, &lastCursorY);
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        rightHeld = action == GLFW_PRESS;
    }
}

void panoramagrid::gl::applications::EquirectViewer::scrollCallback(double xoffset, double yoffset) {
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

void panoramagrid::gl::applications::EquirectViewer::loadTexture() {
    node->getMaterial()->setTexture(cv::imread(inputFile));
    getRenderer()->render(node);
    getRenderer()->loadTexture(node->getMaterial());
}

void panoramagrid::gl::applications::EquirectViewer::initContext() {
    GlApplication::initContext();
    nodes.push_back(node);

    glfwGetCursorPos(window, &lastCursorX, &lastCursorY);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSwapInterval(1);
}

void panoramagrid::gl::applications::EquirectViewer::run() {
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

void panoramagrid::gl::applications::EquirectViewer::parseArgs(int argc, char **argv) {
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
