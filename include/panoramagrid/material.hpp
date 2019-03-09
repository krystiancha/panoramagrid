#ifndef PANORAMAGRID_MATERIAL_HPP
#define PANORAMAGRID_MATERIAL_HPP


#include <opencv2/core/mat.hpp>

namespace panoramagrid {

    class Material {
    public:
        virtual const cv::Mat &getTexture() const = 0;

        virtual void setTexture(const cv::Mat &texture) = 0;

        virtual bool isCubemap() = 0;
    };

}


#endif //PANORAMAGRID_MATERIAL_HPP
