#include <cmath>

#include "common4test.hpp"

double order_floor(double value, int order) {
    return std::floor(value * std::pow(10, order-1)) / std::pow(10, order-1);
}
