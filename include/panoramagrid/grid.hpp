#ifndef PANORAMAGRID_GRID_HPP
#define PANORAMAGRID_GRID_HPP


#include <boost/qvm/vec.hpp>
#include <opencv2/opencv.hpp>
#include <array>
#include <zip.h>

namespace panoramagrid {

    class Grid {
    public:
        void load(std::string path);

        cv::Mat nearest(boost::qvm::vec<float, 3> point);

    private:
        std::vector<boost::qvm::vec<float, 3>> points;
        zip_t *archive;

        static boost::qvm::vec<float, 3> filenameToPoint(std::string filename);
    };

}


#endif //PANORAMAGRID_GRID_HPP
