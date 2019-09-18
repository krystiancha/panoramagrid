#include <panoramagrid/grid.hpp>

int main() {
    panoramagrid::Grid grid;

    grid.open("/home/protecto/CLionProjects/panoramagrid/share/panoramagrid/grid.zip");
    grid.get({10, 10, 10});
    grid.close();
}
