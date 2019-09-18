#include <panoramagrid/panoramagrid.hpp>
#include <panoramagrid/gl/applications/equirectviewer.hpp>

namespace pg = panoramagrid;

int main(int argc, char *argv[]) {
    pg::gl::applications::EquirectViewer app;
    app.parseArgs(argc, argv);

    pg::gl::applications::EquirectViewer::initGlfw();
    app.initContext();

    app.run();

    return 0;
}
