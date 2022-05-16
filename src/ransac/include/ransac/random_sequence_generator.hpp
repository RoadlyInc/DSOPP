#ifndef DSOPP_RANSAC_RANDOM_SEQUENCE_GENERATOR_HPP_
#define DSOPP_RANSAC_RANDOM_SEQUENCE_GENERATOR_HPP_

#include <array>
#include <random>

namespace dsopp::ransac {

/**
 * Function to generate sequence of N random number less than max
 *
 * @tparam N number of numbers
 * @param max upper limit
 * @retrun sequence distinct of random numbers
 */
template <size_t N>
std::array<std::size_t, N> generateRandomSequence(const size_t max) {
  std::array<std::size_t, N> sequence;
  sequence[0] = static_cast<size_t>(rand()) % max;

  for (size_t i = 1; i < N; ++i) {
    size_t next_element = static_cast<size_t>(rand()) % max;

    auto begin = sequence.begin();
    auto end = sequence.begin() + i;

    while (std::find_if(begin, end, [&next_element](size_t v) { return v == next_element; }) != end) {
      next_element = static_cast<size_t>(rand()) % max;
    }

    sequence[i] = next_element;
  }
  return sequence;
}

}  // namespace dsopp::ransac

#endif  // DSOPP_RANSAC_RANDOM_SEQUENCE_GENERATOR_HPP_
