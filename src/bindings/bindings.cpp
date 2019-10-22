#include "bindings.h"

using namespace openravepy; // toPyArray

int main() {
    const std::vector<double> v {1, 2, 3};
    openravepy::bpndarray pyv = toPyArray(v);
    return 0;
}