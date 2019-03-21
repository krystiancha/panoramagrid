#include <panoramagrid/uvmaterial.hpp>

namespace panoramagrid {

    const cv::Mat &UvMaterial::getTexture() const {
        return texture;
    }

    void UvMaterial::setTexture(const cv::Mat &texture) {
        UvMaterial::texture = texture;
    }

    bool UvMaterial::isCubemap() {
        return false;
    }
}
