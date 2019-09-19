#include <panoramagrid/gridpy.hpp>

Grid::Point tupleToPoint(const p::tuple &point) {
    return Grid::Point{
            p::extract<float>(point[0]),
            p::extract<float>(point[1]),
            p::extract<float>(point[2]),
    };
}

p::tuple pointToTuple(Grid::Point point) {
    return p::make_tuple(point[0], point[1], point[2]);
}

np::ndarray matToNdarray(const cv::Mat &mat) {
    return np::from_data(
            mat.data, np::dtype::get_builtin<unsigned char>(),
            p::make_tuple(mat.rows, mat.cols, 3),
            p::make_tuple(mat.cols * 3, 3, 1),
            p::object()
    ).copy();
}

const cv::Mat ndarrayToMat(const boost::python::numpy::ndarray &ndarray) {
    auto shape = ndarray.get_shape();
    if (shape == 0) {
        return cv::Mat();
    }
    return cv::Mat(shape[0], shape[1], CV_8UC3, ndarray.get_data());
}

p::tuple get(Grid self, const p::tuple &point) {
    std::pair<Grid::Point, cv::Mat> ret = self.get(tupleToPoint(point));

    return p::make_tuple(pointToTuple(ret.first), matToNdarray(ret.second));
}

void set(Grid self, const p::tuple &point, np::ndarray &mat) {
    self.set(tupleToPoint(point), ndarrayToMat(mat));
}

p::list list(Grid self) {
    p::list list;

    for (const auto &el : self.list()) {
        list.append(p::make_tuple(pointToTuple(el.first), el.second));
    }

    return list;
}


//

//
//boost::python::numpy::ndarray nearest(panoramagrid::Grid &self, boost::python::tuple &point) {
//    cv::Mat img = self.nearest(tupleToPoint(point));
//    return matToNdarray(img);
//}
//
//void set(panoramagrid::Grid &self, p::tuple &point, np::ndarray &img) {
//    self.set(tupleToPoint(point), ndarrayToMat(img));
//}
//
//p::list getPoints(panoramagrid::Grid &self) {
//    p::list list;
//
//    for (const auto &point : self.getPoints()) {
//        list.append<p::tuple>(p::make_tuple(
//            point.a[0], point.a[1], point.a[2]
//        ));
//    }
//
//    return list;
//}