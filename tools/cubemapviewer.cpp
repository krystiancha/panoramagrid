#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <opencv2/imgcodecs.hpp>
#include <panoramagrid/panoramagrid.hpp>
#include <panoramagrid//gl/gl.hpp>


namespace pg = panoramagrid;

int main(int argc, char *argv[]) {
    const int width = 1280;
    const int height = 720;

    GLFWwindow *window = pg::gl::createContext(width, height, "Cubemap Viewer");

    pg::AppState state(
        width,
        height,
        std::make_shared<pg::gl::GlRenderer>(width, height, std::make_shared<pg::Camera>())
    );
    glfwGetCursorPos(window, &state.lastXpos, &state.lastYpos);
    glfwSetWindowUserPointer(window, &state);

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPosCallback(window, pg::gl::cursorPosCallback);
    glfwSetScrollCallback(window, pg::gl::scrollCallback);

    auto node = std::make_shared<pg::Node>(std::make_shared<pg::CubeMesh>(), std::make_shared<pg::CubemapMaterial>());

    node->getMaterial()->setTexture(cv::imread(argv[1]));

    state.renderer->render(node);
    state.renderer->loadTexture(node->getMaterial());

    glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        state.renderer->render(node);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    pg::gl::destroyContext(window);

    return 0;
}
