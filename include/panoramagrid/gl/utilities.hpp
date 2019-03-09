#ifndef PANORAMAGRID_GLUTILITIES_HPP
#define PANORAMAGRID_GLUTILITIES_HPP


#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <string>
#include <stdexcept>
#include <panoramagrid/utilities.hpp>
#include <cmath>

namespace panoramagrid::gl {

    GLFWwindow *createContext(int width, int height, const std::string &title) {
        GLFWwindow *window;

        if (!glfwInit()) {
            throw std::logic_error("Could not initialize opengl context.");
        }

        glfwSetErrorCallback([](int error, const char *description) {
            throw std::logic_error((std::string) "Error: " + description + "\n");
        });

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
        if (!window) {
            throw std::logic_error("Could not initialize opengl context.");
        }

        glfwSetKeyCallback(window, [](GLFWwindow *window, int key, int scancode, int action, int mods) {
            if ((key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) && action == GLFW_PRESS) {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
        });

        glfwMakeContextCurrent(window);

        gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);

        glfwSwapInterval(1);

        return window;
    }

    void destroyContext(GLFWwindow *window) {
        glfwDestroyWindow(window);

        glfwTerminate();
    }

    void cursorPosCallback(GLFWwindow *window, double xpos, double ypos) {
        auto *windowState = static_cast<AppState *>(glfwGetWindowUserPointer(window));

        auto rpy = windowState->renderer->getCamera()->getOrientation();

        windowState->renderer->getCamera()->setOrientation({
            0,
            panoramagrid::constrain<float>(
                static_cast<float>(rpy[1] - (ypos - windowState->lastYpos) * windowState->mouseSensitivity),
                -M_PI_2f32 + std::numeric_limits<float>::epsilon(),
                M_PI_2f32 - std::numeric_limits<float>::epsilon()
            ),
            static_cast<float>(rpy[2] - (xpos - windowState->lastXpos) * windowState->mouseSensitivity)
        });

        windowState->lastXpos = xpos;
        windowState->lastYpos = ypos;
    }

    void scrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
        auto *windowState = static_cast<AppState *>(glfwGetWindowUserPointer(window));

        windowState->renderer->getCamera()->setFov(
            constrain<float>(
                static_cast<float>(
                    windowState->renderer->getCamera()->getFov() - yoffset * windowState->scrollSensitivity
                ),
                std::numeric_limits<float>::epsilon(),
                M_PIf32 - std::numeric_limits<float>::epsilon()
            )
        );
    }

}

#endif //PANORAMAGRID_GLUTILITIES_HPP
