#ifndef PANORAMAGRID_CubemapMaterial_HPP
#define PANORAMAGRID_CubemapMaterial_HPP


#include <panoramagrid/material.hpp>

namespace panoramagrid {

    class CubemapMaterial : public Material {
    public:
        const cv::Mat &getTexture() const override;

        void setTexture(const cv::Mat &texture) override;

        bool isCubemap() override;

    private:
        cv::Mat texture;
    };

}


#endif //PANORAMAGRID_CubemapMaterial_HPP
