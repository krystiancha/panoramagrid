#ifndef PANORAMAGRID_GRID_HPP
#define PANORAMAGRID_GRID_HPP


#include <opencv2/opencv.hpp>
#include <array>
#include <zip.h>
#include <thread>
#include <mutex>
#include <unordered_set>
#include <condition_variable>

namespace panoramagrid {

    class Grid {
    public:
        typedef std::array<float, 3> Point;
        typedef unsigned long Index;
        typedef zip_uint64_t Size;

        Grid();

        void open(const std::string &path);

        void close();

        void startThread();

        std::pair<Point, cv::Mat> get(Point point);

        std::vector<Point> list();

    private:
        std::string path;
        zip_t *archive = nullptr;
        std::vector<Point> points;
        std::vector<Size> sizes;
        Point lastPoint = {0, 0, 0};
        std::mutex ready;
        std::mutex newPosition;
        std::unordered_map<Index, cv::Mat> cache;
        std::unordered_map<Index, std::mutex> cacheMutexes;
        std::thread thread;
        bool stopThread = false;

        Grid::Index findClosestExcept(Point point, const std::unordered_set<Index>& exceptions = {});
        void cacheThread();
        void load(Index index);

        static Point filenameToPoint(std::string filename);
        static std::string pointToFilename(Point point);
    };

}


#endif //PANORAMAGRID_GRID_HPP
