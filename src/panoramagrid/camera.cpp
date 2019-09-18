#include <panoramagrid/camera.hpp>

namespace panoramagrid {

    std::array<float, 3> rotateVector(const std::array<float, 3> &vector, const std::array<float, 4> &quaternion);
    std::array<float, 4> multiplyQuaternions(std::array<float, 4> a, std::array<float, 4> b);
    std::array<float, 4> rpyToQuaternion(float yaw, float pitch, float roll);
    std::array<float, 4> inverse(std::array<float, 4> quaternion);

    Camera::Camera(float aspectRatio, float fov, const std::array<float, 3> &position,
        const std::array<float, 4> &orientation)
        : position(position), orientation(orientation), fov(fov), aspectRatio(aspectRatio) {}

    void Camera::setRelativePosition(const std::array<float, 3> &position) {
        auto currentPosition = getPosition();
        auto absPosition = rotateVector(position, inverse(orientation));
        setPosition({
            currentPosition[0] + absPosition[0],
            currentPosition[1] + absPosition[1],
            currentPosition[2] + absPosition[2],
        });
    }

    void Camera::setRelativeOrientation(const std::array<float, 3> &rpy) {
        std::array<float, 4> q1 = rpyToQuaternion(0, 0, rpy[2]); // pitch
        std::array<float, 4> q2 = rpyToQuaternion(0, rpy[1], 0); // yaw
        std::array<float, 4> q3 = rpyToQuaternion(rpy[0], 0, 0); // roll

        setOrientation(multiplyQuaternions(q3, getOrientation()));
        setOrientation(multiplyQuaternions(getOrientation(), q2));
        setOrientation(multiplyQuaternions(q1, getOrientation()));
    }

    const std::array<float, 3> &Camera::getPosition() const {
        return position;
    }

    void Camera::setPosition(const std::array<float, 3> &position) {
        Camera::position = position;
    }

    const std::array<float, 4> &Camera::getOrientation() const {
        return orientation;
    }

    void Camera::setOrientation(const std::array<float, 4> &orientation) {
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

    std::array<float, 3> rotateVector(const std::array<float, 3> &vector, const std::array<float, 4> &quaternion) {
        float s = quaternion[3];
        std::array<float, 3> u {quaternion[0], quaternion[1], quaternion[2]};

        float dot1 = u[0] * vector[0] + u[1] * vector[1] + u[2] * vector[2];
        std::array<float, 3> v1 {2 * dot1 * u[0], 2 * dot1 * u[1], 2 * dot1 * u[2]};

        float dot2 = u[0] * u[0] + u[1] * u[1] +u[2] * u[2];
        std::array<float, 3> v2 {(s * s - dot2) * vector[0], (s * s - dot2) * vector[1], (s * s - dot2) * vector[2]};

        std::array<float, 3> v3 {
            2 * s * (u[1] * vector[2] - vector[1] * u[2]),
            2 * s * (vector[0] * u[2] - u[0] * vector[2]),
            2 * s * (u[0] * vector[1] - vector[0] * u[1]),
        };

        return {
            v1[0] + v2[0] + v3[0],
            v1[1] + v2[1] + v3[1],
            v1[2] + v2[2] + v3[2],
        };
    }

    std::array<float, 4> multiplyQuaternions(std::array<float, 4> a, std::array<float, 4> b) {
        return {
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
            -a[0] * b[2] + a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[1] - a[1] * b[0] + a[2] * b[3] + a[3] * b[2],
            -a[0] * b[0] - a[1] * b[1] - a[2] * b[2] + a[3] * b[3],
        };
    }


    std::array<float, 4> rpyToQuaternion(float yaw, float pitch, float roll) // yaw (Z), pitch (Y), roll (X)
    {
        // Abbreviations for the various angular functions
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);

        std::array<float, 4> q {
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr,
            cy * cp * cr + sy * sp * sr,
        };

        return q;
    }

    float norm(std::array<float, 4> quaternion) {
        return sqrtf(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    }

    std::array<float, 4> conjugate(std::array<float, 4> quaternion) {
        return {
            -quaternion[0],
            -quaternion[1],
            -quaternion[2],
            quaternion[3],
        };
    }

    std::array<float, 4> inverse(std::array<float, 4> quaternion) {
        float absolute = norm(quaternion);
        absolute *= absolute;
        absolute = 1 / absolute;

        std::array<float, 4> conj = conjugate(quaternion);

        return {
            conj[0] * absolute,
            conj[1] * absolute,
            conj[2] * absolute,
            conj[3] * absolute,
        };
    }

}
