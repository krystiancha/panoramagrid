#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/find.hpp>
#include <boost/qvm/all.hpp>
#include <panoramagrid/grid.hpp>

namespace panoramagrid {

    void Grid::load(std::string path) {
        int error;
        archive = zip_open(path.c_str(), ZIP_RDONLY, &error);

        if (!error) {
            throw std::runtime_error("Could not open the archive");
        }

        zip_int64_t entries = zip_get_num_entries(archive, 0);

        points.resize(static_cast<unsigned long>(entries));

        for (zip_uint64_t i = 0; i < entries; ++i) {
            zip_stat_t sb;

            if (zip_stat_index(archive, i, ZIP_STAT_NAME, &sb)) {
                throw std::runtime_error("Could not get info about the archive");
            }

            points[i] = filenameToPoint(sb.name);
        }
    }

    cv::Mat Grid::nearest(boost::qvm::vec<float, 3> point) {
        if (points.empty()) {
            throw std::runtime_error("There are no points on the grid");
        }

        float closestDistance = boost::qvm::mag(points[0] - point);
        int closestIndex = 0;

        for (int i = 1; i < points.size(); ++i) {
            float distance = boost::qvm::mag(points[i] - point);
            if (distance < closestDistance) {
                closestDistance = distance;
                closestIndex = i;
            }
        }

        zip_stat_t sb;
        zip_stat_index(archive, static_cast<zip_uint64_t>(closestIndex), ZIP_STAT_SIZE, &sb);

        auto buffer = new unsigned char[sb.size];

        zip_file_t *file = zip_fopen_index(archive, static_cast<zip_uint64_t>(closestIndex), 0);

        zip_fread(file, buffer, sb.size);

        cv::Mat mat = cv::imdecode(std::vector<unsigned char>(buffer, buffer + sb.size), cv::IMREAD_COLOR);

        delete[] buffer;

        return mat;
    }

    boost::qvm::vec<float, 3> Grid::filenameToPoint(std::string filename) {
        boost::algorithm::replace_all(filename, "_", " ");
        auto end = filename.rfind('.');

        std::stringstream ss(filename.substr(0, end));
        boost::qvm::vec<float, 3> point{};
        ss >> point.a[0] >> point.a[1] >> point.a[2];
        return point;
    }

}
