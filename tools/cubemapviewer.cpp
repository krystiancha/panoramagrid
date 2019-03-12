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

#include <panoramagrid/panoramagrid.hpp>
#include <panoramagrid/gl/applications/cubemapviewer.hpp>

namespace pg = panoramagrid;

int main(int argc, char *argv[]) {
    pg::gl::applications::CubemapViewer app;
    app.parseArgs(argc, argv);

    pg::gl::applications::CubemapViewer::initGlfw();
    app.initContext();

    app.run();

    return 0;
}
