#ifndef DSOPP_CALCULATE_PIXELINFO_HPP
#define DSOPP_CALCULATE_PIXELINFO_HPP

namespace dsopp::features {

template <int C, typename T>
void calculate_pixelinfo(const T *input, T *output, const int width, const int height);

}

#endif  // DSOPP_CALCULATE_PIXELINFO_HPP