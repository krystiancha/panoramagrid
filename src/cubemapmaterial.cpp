#include <panoramagrid/cubemapmaterial.hpp>

namespace panoramagrid {

    const cv::Mat &CubemapMaterial::getTexture() const {
        return texture;
    }

    void CubemapMaterial::setTexture(const cv::Mat &texture) {
        CubemapMaterial::texture = texture;
    }

    bool CubemapMaterial::isCubemap() {
        return true;
    }

}
