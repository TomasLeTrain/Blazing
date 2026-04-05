#include "blazing/latex_utils.hpp"

namespace blazing {
// prints pair of floats as (x,y) coordinate pair
// NOTE: does not leave newline
void printPairAsLatex(std::pair<float, float> data) {
    // make sure it doesn't print in a weird format
    // doesn't set precision in case user might have a desired precision
    std::cout << std::fixed;
    std::cout << "\\left(" << data.first << "," << data.second << "\\right)";
}

// calls nu  pair of floats as (x,y) coordinate pair
void printPairListAsLatex(
  size_t size,
  std::function<std::pair<float, float>(size_t)> num_func) {
    std::cout << "\\left[";
    for (size_t i = 0; i < size; i++) {
        printPairAsLatex(num_func(i));

        if (i < size - 1) std::cout << ",";
    }
    std::cout << "\\right]" << std::endl;
}

// prints num_func as a pair size # of times, adds \left[ and
// \right] as well as commas in bewtween
void printListAsLatex(size_t size, std::function<float(size_t)> num_func) {
    std::cout << "\\left[";
    for (size_t i = 0; i < size; i++) {
        std::cout << num_func(i);
        // doesn't print comma for last point
        if (i < size - 1) std::cout << ",";
    }
    std::cout << "\\right]" << std::endl;
}
} // namespace blazing
