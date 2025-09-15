#include "fake_moma/moma_param.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(moma_param, m) 
{
    py::class_<MomaParam>(m, "MomaParam")
    .def(py::init())
    .def("getColliMarkerArray", &MomaParam::getColliMarkerArray)
    .def("getJointLimitsMin", &MomaParam::getJointLimitsMin)
    .def("getJointLimitsMax", &MomaParam::getJointLimitsMax)
    .def("getJointVelLimits", &MomaParam::getJointVelLimits)
    .def("getBallsGrids", &MomaParam::getBallsGrids)
    .def("getFKPose", &MomaParam::getFKPose)
    .def("getEEGrads", &MomaParam::getEEGrads)
    .def_readonly("chassis_height", &MomaParam::chassis_height)
    .def_readonly("chassis_colli_radius", &MomaParam::chassis_colli_radius)
    .def_readonly("max_v", &MomaParam::max_v)
    .def_readonly("max_a", &MomaParam::max_a)
    .def_readonly("max_w", &MomaParam::max_w)
    .def_readonly("max_dw", &MomaParam::max_dw)
    .def_property_readonly("colli_length", [](const MomaParam &a) { return a.colli_length; })
    .def_property_readonly("colli_points", [](const MomaParam &a) { return a.colli_points; })
    .def_property_readonly("colli_point_radius", [](const MomaParam &a) { return a.colli_point_radius; })
    .def_property_readonly("chassis_colli_radius", [](const MomaParam &a) { return a.chassis_colli_radius; })
    .def_property_readonly("collision_matrix", [](const MomaParam &a) { return a.collision_matrix; })
    .def_property_readonly("relative_R", [](const MomaParam &a) { return RowMatrixXd(a.relative_R); })
    .def_property_readonly("relative_t", [](const MomaParam &a) { return a.relative_t; })
    .def("__repr__",
        [](const MomaParam &a) {
            return "<moma_param.MomaParam>";
        }
    );
}