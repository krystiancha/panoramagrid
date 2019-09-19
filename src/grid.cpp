#include <boost/algorithm/string/replace.hpp>
#include <boost/qvm/all.hpp>
#include <panoramagrid/grid.hpp>

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

    void Grid::flush() {
        close();
        open(path);
    }

    std::pair<Grid::Point, cv::Mat> Grid::get(Point point) {
        if (archive == nullptr) {
            throw std::runtime_error("open() has not been called");
        }

        std::vector<std::pair<Grid::Point, unsigned long>> points = list();

        zip_uint64_t closestIndex = 0;
        Point closestPoint;
        long closestSize = 0;
        float closestDistance = std::numeric_limits<float>::max();
        for (std::vector<Grid::Point>::size_type i = 0; i < points.size(); ++i) {
            if (points[i].second == 0) {
                continue;
            }
            Point current = points[i].first;
            float distance = sqrtf(
                    (current[0] - point[0]) * (current[0] - point[0])
                    + (current[1] - point[1]) * (current[1] - point[1])
                    + (current[2] - point[2]) * (current[2] - point[2])
            );
            if (distance < closestDistance) {
                closestIndex = i;
                closestPoint = current;
                closestSize = points[i].second;
                closestDistance = distance;
            }
        }

        zip_file_t *file = zip_fopen_index(archive, closestIndex, 0);
        if (file == nullptr) {
            throw std::runtime_error(
                    "Could not open file " + std::to_string(closestIndex) + ": " + std::string(zip_strerror(archive))
            );
        }

        std::vector<unsigned char> buffer(closestSize, '\0');
        zip_int64_t bytesRead = zip_fread(file, buffer.data(), closestSize);
        if (bytesRead != closestSize) {
            throw std::runtime_error(
                    "Error reading file" + std::to_string(closestIndex)
                    + ": Read " + std::to_string(bytesRead) + " bytes instead of expected "
                    + std::to_string(closestSize) + " bytes"
            );
        }

        //TODO: chceck for fail
        zip_fclose(file);

        cv::Mat mat = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
        if (mat.data == nullptr) {
            throw std::runtime_error("Could not decode file " + std::to_string(closestIndex));
        }

        return std::make_pair(closestPoint, mat);
    }

    unsigned long Grid::set(Point point, const cv::Mat &mat) {
        if (archive == nullptr) {
            throw std::runtime_error("open() has not been called");
        }

        std::vector<unsigned char> buffer;
        if (!mat.empty()) {
            if (!cv::imencode(".jpg", mat, buffer)) {
                throw std::runtime_error("Could not encode file");
            }
        }

        zip_source_t *source = zip_source_buffer(archive, buffer.data(), buffer.size(), 0);
        if (source == nullptr) {
            throw std::runtime_error("Could not create source: " + std::string(zip_strerror(archive)));
        }

        long index = zip_file_add(archive, pointToFilename(point).c_str(), source, ZIP_FL_OVERWRITE);
        if (index == -1) {
            zip_source_free(source);
            throw std::runtime_error("Could not add file: " + std::string(zip_strerror(archive)));
        }

        return index;
    }

    std::vector<std::pair<Grid::Point, unsigned long>> Grid::list() {
        if (archive == nullptr) {
            throw std::runtime_error("open() has not been called");
        }

        auto entries = static_cast<zip_uint64_t>(zip_get_num_entries(archive, 0));
        std::vector<std::pair<Grid::Point, unsigned long>> points(entries);
        for (zip_uint64_t i = 0; i < entries; ++i) {
            zip_stat_t sb;
            if (zip_stat_index(archive, i, ZIP_STAT_NAME | ZIP_STAT_COMP_SIZE, &sb) != 0) {
                throw std::runtime_error(
                        "Could not info about file " + std::to_string(i) + ": " + std::string(zip_strerror(archive))
                );
            }

            points[i] = std::make_pair(filenameToPoint(sb.name), sb.comp_size);
        }

        return points;
    }

    void Grid::remove(unsigned long index) {
        if (archive == nullptr) {
            throw std::runtime_error("open() has not been called");
        }

        if (zip_delete(archive, index) != 0) {
            throw std::runtime_error(
                    "Could not delete file " + std::to_string(index) + ": " + std::string(zip_strerror(archive))
            );
        }
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

}
