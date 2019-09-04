#ifndef PANORAMAGRID_GRIDPY_HPP
#define PANORAMAGRID_GRIDPY_HPP

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <panoramagrid/grid.hpp>

namespace p = boost::python;
namespace np = boost::python::numpy;
using namespace panoramagrid;

p::tuple get(Grid self, const p::tuple& point);
void set(Grid self, const p::tuple& point, np::ndarray &mat);
p::list list(Grid self);

BOOST_PYTHON_MODULE(gridpy)
{
    np::initialize();
    p::class_<panoramagrid::Grid>("Grid")
        .def("open", &Grid::open)
        .def("close", &Grid::close)
        .def("flush", &Grid::flush)
        .def("get", &get)
        .def("set", &set)
        .def("list", &list)
        .def("remove", &Grid::remove)
    ;
}

#endif //PANORAMAGRID_GRIDPY_HPP
