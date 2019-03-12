/*!
 * The cubemapstitch application joins six cube face images to a single cubemap using a layout that is compatible with
 * the panoramagrid API.
 */

#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>

namespace po = boost::program_options;

int main(int argc, char *argv[]) {
    const std::array<std::string, 6> sideNames = {"right", "left", "top", "bottom", "front", "back"};
    const std::array<std::pair<int, int>, 6> sideLocation{
        std::make_pair<int, int>(2, 1),
        std::make_pair<int, int>(0, 1),
        std::make_pair<int, int>(1, 0),
        std::make_pair<int, int>(1, 2),
        std::make_pair<int, int>(1, 1),
        std::make_pair<int, int>(3, 1),
    };

    po::options_description options;

    po::options_description standardOptions("Allowed options");
    standardOptions.add_options()
        ("help,h", "produce help message")
        ("output,o", po::value<std::string>()->default_value("cubemap.jpg"), "Name of the output file");
    options.add(standardOptions);

    po::options_description hiddenOptions("Hidden options");
    po::positional_options_description positionalOptions;
    for (const auto &sideName : sideNames) {
        hiddenOptions.add_options()(sideName.c_str(), po::value<std::string>()->required());
        positionalOptions.add(sideName.c_str(), 1);
    }
    options.add(hiddenOptions);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options).positional(positionalOptions).run(), vm);

    bool printHelp = false;
    try {
        po::notify(vm);
        printHelp = static_cast<bool>(vm.count("help"));
    } catch (boost::program_options::required_option &e) {
        printHelp = true;
    }

    if (printHelp) {
        std::cout << "Usage: cubemapstitch <right> <left> <top> <bottom> <front> <back>" << std::endl;
        std::cout << standardOptions << std::endl;
        return 0;
    }

    cv::Mat sides[6];https://github.com/
    int sideDim;

    for (int i = 0; i < 6; ++i) {
        auto filename = vm[sideNames[i]].as<std::string>();
        sides[i] = cv::imread(filename);
        if (sides[i].empty()) {
            throw std::runtime_error("Could not open file: " + filename);
        }
        if (i == 0) {
            sideDim = sides[i].cols;
            if (sides[i].rows != sideDim) {
                throw std::runtime_error("Cube sides have to be square");
            }
        } else if (sides[i].cols != sideDim || sides[i].rows != sideDim) {
            throw std::runtime_error("All cube sides have to be the same size");
        }
    }

    cv::Mat cubemap(3 * sideDim, 4 * sideDim, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < 6; ++i) {
        sides[i].copyTo(
            cubemap(cv::Rect(sideLocation[i].first * sideDim, sideLocation[i].second * sideDim, sideDim, sideDim)));
    }

    cv::imwrite(vm["output"].as<std::string>(), cubemap);

}