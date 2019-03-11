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
