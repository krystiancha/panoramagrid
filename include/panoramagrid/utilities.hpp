#include <utility>

#ifndef PANORAMAGRID_UTILITIES_HPP
#define PANORAMAGRID_UTILITIES_HPP


namespace panoramagrid {

    class AppState {
    public:
        AppState(int width, int height, std::shared_ptr<Renderer> renderer)
            : lastXpos(0), lastYpos(0), lastYScroll(0), renderer(std::move(renderer)) {}

        const float mouseSensitivity = 0.002, scrollSensitivity = 0.05;
        double lastXpos, lastYpos, lastYScroll;
        std::shared_ptr<Renderer> renderer;
    };

    template<typename T>
    T constrain(T x, T a, T b) {
        return x < a ? a : (x > b ? b : x);
    }

}

#endif //PANORAMAGRID_UTILITIES_HPP
