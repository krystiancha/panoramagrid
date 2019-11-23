#include <boost/algorithm/string/replace.hpp>
#include <boost/qvm/all.hpp>
#include <panoramagrid/grid.hpp>
#include <thread>
#include <chrono>

namespace panoramagrid {
    void Grid::open(const std::string &path) {
        if (archive != nullptr) {
            throw std::runtime_error("Already open");
        }

        Grid::path = path;

        int errorp;
        archive = zip_open(path.c_str(), ZIP_CREATE, &errorp);
        if (archive == nullptr) {
            throw std::runtime_error("Could not open the archive");
        }

        auto entries = static_cast<zip_uint64_t>(zip_get_num_entries(archive, 0));
        for (zip_uint64_t i = 0; i < entries; ++i) {
            zip_stat_t sb;
            if (zip_stat_index(archive, i, ZIP_STAT_NAME | ZIP_STAT_COMP_SIZE, &sb) != 0) {
                throw std::runtime_error(
                        "Could not info about file " + std::to_string(i) + ": " + std::string(zip_strerror(archive))
                );
            }

            if (sb.comp_size > 0) {
                points.emplace_back(filenameToPoint(sb.name));
                sizes.emplace_back(sb.comp_size);
            }
        }
    }

    void Grid::close() {
        if (archive == nullptr) {
            throw std::runtime_error("open() has not been called");
        }

        if (zip_close(archive) != 0) {
            throw std::runtime_error("Could not close: " + std::string(zip_strerror(archive)));
        }
        archive = nullptr;
    }

    void Grid::cacheThread() {
        cacheMutexes.clear();
        for (Index i = 0; i < points.size(); ++i) {
            cacheMutexes[i].lock();
        }
        ready.unlock();
        while (!stopThread) {
            newPosition.lock();

            // Find points that should be in the cache
            std::unordered_set<Index> closest = {};
            for (int i = 0; i < 4; ++i) {
                closest.insert(findClosestExcept(lastPoint, closest));
            }

            // Add them to the cache
            for (const auto &index : closest) {
                if (!cache.count(index)) {
                    load(index);
                    cacheMutexes[index].unlock();
                }
            }

            // Remove points that should not be in the cache
            for (auto it = cache.begin(); it != cache.end(); ) {
                if (!closest.count(it->first)) {
                    cacheMutexes[it->first].lock();
                    it = cache.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    std::pair<Grid::Point, cv::Mat> Grid::get(Point point) {
        if (archive == nullptr) {
            throw std::runtime_error("open() has not been called");
        }

        lastPoint = point;
        newPosition.unlock();

        Index closest = findClosestExcept(point);

        std::mutex &mutex = cacheMutexes[closest];

        std::lock_guard<std::mutex> lock(ready);
        if (mutex.try_lock()) {
            const cv::Mat &mat = cache.at(closest);
            mutex.unlock();

            return {points[closest], mat};
        }

        std::cerr << "Can't keep up with decoding images..." << std::endl;

        mutex.lock();
        const cv::Mat &mat = cache.at(closest);
        mutex.unlock();

        return {points[closest], mat};
    }

    std::vector<Grid::Point> Grid::list() {
        if (archive == nullptr) {
            throw std::runtime_error("open() has not been called");
        }

        return points;
    }

    Grid::Point Grid::filenameToPoint(std::string filename) {
        boost::algorithm::replace_all(filename, "_", " ");
        auto end = filename.rfind('.');

        std::stringstream ss(filename.substr(0, end));
        Grid::Point point;
        ss >> point[0] >> point[1] >> point[2];
        return point;
    }

    std::string Grid::pointToFilename(Grid::Point point) {
        std::stringstream ss;
        ss << point[0] << "_" << point[1] << "_" << point[2] << ".jpg";
        return ss.str();
    }

    Grid::Index Grid::findClosestExcept(Point point, const std::unordered_set<Index>& exceptions) {
        Index closestIndex = 0;
        float closestDistance = std::numeric_limits<float>::max();

        for (Index i = 0; i < points.size(); ++i) {
            if (exceptions.count(i)) {
                continue;
            }
            Point current = points[i];
            float distance = sqrtf(
                    (current[0] - point[0]) * (current[0] - point[0])
                    + (current[1] - point[1]) * (current[1] - point[1])
                    + (current[2] - point[2]) * (current[2] - point[2])
            );
            if (distance < closestDistance) {
                closestIndex = i;
                closestDistance = distance;
            }
        }

        return closestIndex;
    }

    void Grid::load(Index index) {
        zip_file_t *file = zip_fopen_index(archive, index, 0);
        if (file == nullptr) {
            throw std::runtime_error(
                    "Could not open file " + std::to_string(index) + ": " +
                    std::string(zip_strerror(archive))
            );
        }

        std::vector<unsigned char> buffer(sizes[index], '\0');
        zip_int64_t bytesRead = zip_fread(file, buffer.data(), sizes[index]);
        if (bytesRead != sizes[index]) {
            throw std::runtime_error(
                    "Error reading file" + std::to_string(index)
                    + ": Read " + std::to_string(bytesRead) + " bytes instead of expected "
                    + std::to_string(sizes[index]) + " bytes"
            );
        }

        //TODO: chceck for fail
        zip_fclose(file);

        cache[index] = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
        if (cache[index].data == nullptr) {
            throw std::runtime_error("Could not decode file " + std::to_string(index));
        }
    }

    void Grid::startThread() {
        thread = std::thread([this]() {
            this->cacheThread();
        });
    }

    Grid::Grid() {
        ready.lock();
    }

}
