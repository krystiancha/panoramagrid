#ifndef PANORAMAGRID_GRID_HPP
#define PANORAMAGRID_GRID_HPP


#include <opencv2/opencv.hpp>
#include <array>
#include <zip.h>

namespace panoramagrid {

    class Grid {
    public:
        typedef std::array<float, 3> Point;

        void open(const std::string &path);

        void close();

        void flush();

        std::pair<Point, cv::Mat> get(Point point);

        unsigned long set(Point point, const cv::Mat &mat);

        std::vector<std::pair<Point, unsigned long>> list();

        void remove(unsigned long index);

    private:
        std::string path;
        zip_t *archive = nullptr;

        static Point filenameToPoint(std::string filename);

        static std::string pointToFilename(Point point);
    };

}


#endif //PANORAMAGRID_GRID_HPP
