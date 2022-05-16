#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pydsopp/features/extract_tracking_features.hpp"

PYBIND11_MODULE(pydsopp, pydsopp) {
  pydsopp.doc() = "DSOPP Python binding";

  auto features = pydsopp.def_submodule("features", "Features");
  features.def("extract_tracking_features", &dsopp::pydsopp::features::extract_tracking_features,
               "Extracts tracking features from BGR image", pybind11::arg("image"));
}
