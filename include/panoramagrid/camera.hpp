#ifndef PANORAMAGRID_CAMERA_HPP
#define PANORAMAGRID_CAMERA_HPP


#include <cmath>
#include <array>

namespace panoramagrid {

    class Camera {
    public:
        Camera(const Camera &) = default;

        explicit Camera(float aspectRatio = 4.0f / 3.0f, float fov = M_PI_2f32,
            const std::array<float, 3> &position = {0, 0, 0},
            const std::array<float, 3> &orientation = {0, 0, 0});

        const std::array<float, 3> &getPosition() const;

        void setPosition(const std::array<float, 3> &position);

        const std::array<float, 3> &getOrientation() const;

        void setOrientation(const std::array<float, 3> &orientation);

        float getFov() const;

        void setFov(float fov);

        float getAspectRatio() const;

        void setAspectRatio(float aspectRatio);

    private:
        std::array<float, 3> position;
        std::array<float, 3> orientation;
        float fov;
        float aspectRatio;
    };

}


#endif //PANORAMAGRID_CAMERA_HPP
