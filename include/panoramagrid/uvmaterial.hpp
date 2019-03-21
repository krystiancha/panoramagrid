#ifndef PANORAMAGRID_UVMATERIAL_HPP
#define PANORAMAGRID_UVMATERIAL_HPP


#include <panoramagrid/material.hpp>

namespace panoramagrid {

    class UvMaterial : public Material {
    public:
        const cv::Mat &getTexture() const override;

        void setTexture(const cv::Mat &texture) override;

        bool isCubemap() override;

    private:
        cv::Mat texture;
    };

}


#endif //PANORAMAGRID_UVMATERIAL_HPP
