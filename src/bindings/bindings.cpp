#include "bindings.h"

using namespace openravepy; // toPyArray

int main() {
    Py_Initialize();
    boost::python::numpy::initialize();
    std::vector<double> v {1, 2, 3};
    openravepy::bpndarray pyv = toPyArray(v);
    return 0;
}