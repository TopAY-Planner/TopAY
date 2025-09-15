#pragma once

#include "planner/moma_traj_opt.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/embed.h>
#include <fstream>

using namespace nmoma_planner;
namespace py = pybind11;

template <int PWNUM>
std::vector<Eigen::MatrixXd> loadPrimitives(const std::string& filename)
{
    std::vector<Eigen::MatrixXd> matrices;

    py::scoped_interpreter guard{};
    py::module np = py::module::import("numpy");

    std::ifstream file(filename);
    if (!file.good())
        return matrices;

    py::object data = np.attr("load")(filename);
    py::array_t<double> array = data.cast<py::array_t<double>>();
    
    // Ensure the array is of the correct shape
    if (array.ndim() != 3 || array.shape(1) != PWNUM || array.shape(2) != 2) 
    {
        throw std::runtime_error("Array shape error");
    }

    matrices.reserve(array.shape(0));

    auto buf = array.request();
    double* ptr = static_cast<double*>(buf.ptr);

    for (ssize_t i = 0; i < array.shape(0); ++i) 
    {
        Eigen::MatrixXd mat(PWNUM, 2);
        for (ssize_t j = 0; j < PWNUM; ++j) 
        {
            for (ssize_t k = 0; k < 2; ++k) 
            {
                mat(j, k) = ptr[i * PWNUM * 2 + j * 2 + k];
            }
        }
        matrices.push_back(mat);
    }

    return matrices;
}
