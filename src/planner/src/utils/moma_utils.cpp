#include "planner/planner.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace nmoma_planner;
namespace py = pybind11;

PYBIND11_MODULE(moma_utils, m) 
{
    py::class_<MobileTraj>(m, "MobileTraj")
    .def(py::init())
    .def("setTraj", &MobileTraj::setTraj)
    .def("getTotalDuration", &MobileTraj::getTotalDuration)
    .def("getState", &MobileTraj::getState)
    .def("sampleArcPoints", &MobileTraj::sampleArcPoints)
    .def("__repr__",
        [](const MobileTraj &a) {
            return "<moma_utils.MobileTraj>";
        }
    );

    py::class_<MomaTraj>(m, "MomaTraj")
    .def(py::init())
    .def("setTraj", &MomaTraj::setTraj)
    .def("getTotalDuration", &MomaTraj::getTotalDuration)
    .def("getState", &MomaTraj::getState)
    .def("sampleArcPoints", &MomaTraj::sampleArcPoints)
    .def("sampleTimePoints", &MomaTraj::sampleTimePoints)
    .def("__repr__",
        [](const MomaTraj &a) {
            return "<moma_utils.MomaTraj>";
        }
    );
}