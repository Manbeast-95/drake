#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "drake/systems/framework/system.h"
#include "drake/systems/primitives/linear_system.h"

namespace py = pybind11;

PYBIND11_PLUGIN(systems) {
  using drake::systems::System;
  using drake::systems::LinearSystem;

  py::module m("systems",
               "Drake Systems Framework Bindings");

  py::class_<LinearSystem<double>>(
    m, "LinearSystem")
    .def(py::init<  const Eigen::Ref<const Eigen::MatrixXd>&,
  const Eigen::Ref<const Eigen::MatrixXd>&,
  const Eigen::Ref<const Eigen::MatrixXd>&,
  const Eigen::Ref<const Eigen::MatrixXd>&,
  double>(),
    py::arg("A"), py::arg("B"), py::arg("C"), py::arg("D"), py::arg("time_period") = 0.0);

  return m.ptr();
}