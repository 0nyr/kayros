#include <pybind11/pybind11.h>

namespace py = pybind11;

#ifndef KAYROS_VERSION
#define KAYROS_VERSION "0.0.0"
#endif

PYBIND11_MODULE(_core, m) {
    m.doc() = "kayros compiled core: NDCPWLF engine, POD instance/route model, heuristics";
    m.attr("__version__") = KAYROS_VERSION;
}
