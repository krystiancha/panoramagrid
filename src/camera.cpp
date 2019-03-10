#include <panoramagrid/camera.hpp>

namespace panoramagrid {

    Camera::Camera(
        const std::array<float, 3> &position,
        const std::array<float, 3> &orientation,
        float fov,
        float aspectRatio
    ) : position(position), orientation(orientation), fov(fov), aspectRatio(aspectRatio) {}

    const std::array<float, 3> &Camera::getPosition() const {
        return position;
    }

    void Camera::setPosition(const std::array<float, 3> &position) {
        Camera::position = position;
    }

    const std::array<float, 3> &Camera::getOrientation() const {
        return orientation;
    }

    void Camera::setOrientation(const std::array<float, 3> &orientation) {
        Camera::orientation = orientation;
    }

    float Camera::getFov() const {
        return fov;
    }

    void Camera::setFov(float fov) {
        Camera::fov = fov;
    }

    float Camera::getAspectRatio() const {
        return aspectRatio;
    }

    void Camera::setAspectRatio(float aspectRatio) {
        Camera::aspectRatio = aspectRatio;
    }

}