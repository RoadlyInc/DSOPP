#include "features/camera/calculate_pixelinfo.hpp"
#include "common/settings.hpp"

#include <algorithm>
#include <cstdint>

#ifdef __AVX2__
#include <immintrin.h>
#endif

namespace {

inline bool is_aligned(const void *ptr, std::uintptr_t alignment) noexcept {
  auto iptr = reinterpret_cast<std::uintptr_t>(ptr);
  return !(iptr % alignment);
}

#ifdef __AVX2__

static inline __m256d calculate_dy(const __m256d &top_row, const __m256d &bot_row, const __m256d &div) {
  // calculate dy
  // just subtract bottom row from top and divide
  __m256d dy = _mm256_sub_pd(bot_row, top_row);
  dy = _mm256_mul_pd(div, dy);
  return dy;
}

static inline __m256d calculate_dx(const __m256d &left, const __m256d &curr, const __m256d &right, const __m256d &div) {
  // calculate dx
  // 1. mix and permute the elements (6 `double`s) to get two registers where the required subtraction operands are in
  // same positions
  __m256d g1 = _mm256_blend_pd(left, curr, 0b0111);
  __m256d g2 = _mm256_blend_pd(right, curr, 0b1110);
  g1 = _mm256_permute4x64_pd(g1, 0b01001110);
  // 2. subtract and restore order
  __m256d dx = _mm256_sub_pd(g2, g1);
  dx = _mm256_permute4x64_pd(dx, 0b00111001);
  // 3. divide by 2 (or by 1)
  dx = _mm256_mul_pd(div, dx);
  return dx;
}

static inline void process_chunk_avx2_x1(__m256d &v_left, __m256d &v_curr, __m256d &v_right, __m256d &v_top,
                                         __m256d &v_bot, __m256d &out_1, __m256d &out_2, __m256d &out_3, __m256d &div_x,
                                         __m256d &div_y) {
  // a "chunk" looks something like this:
  // ________________________________________________________
  // | ...    |         |         |  v_top  |         | ...
  // | ...    |         | v_left  |  v_curr | v_right | ...
  // | ...    |         |         |  v_bot  |         | ...
  // |
  // , where every cell is 4 input `double` values

  __m256d dx = calculate_dx(v_left, v_curr, v_right, div_x);
  __m256d dy = calculate_dy(v_top, v_bot, div_y);

  // combine the elements into temp vectors to prepare for interleaving
  __m256d t1_1 = _mm256_shuffle_pd(v_curr, dx, 0b0000);
  __m256d t2_1 = _mm256_shuffle_pd(dx, dy, 0b1111);
  __m256d t3_1 = _mm256_shuffle_pd(dy, v_curr, 0b1010);

  // finish interleaving into three output registers
  out_1 = _mm256_permute2f128_pd(t1_1, t3_1, 0b00100000);
  out_2 = _mm256_permute2f128_pd(t1_1, t2_1, 0b00010010);
  out_3 = _mm256_permute2f128_pd(t2_1, t3_1, 0b00010011);
}

static inline void process_cache_line_avx2_x1(__m256d &v_left_1, __m256d &v_curr_1, __m256d &v_right_1,
                                              __m256d &v_top_1, __m256d &v_bot_1, __m256d &v_left_2, __m256d &v_curr_2,
                                              __m256d &v_right_2, __m256d &v_top_2, __m256d &v_bot_2, __m256d &div_x_1,
                                              __m256d &div_x_2, __m256d &div_y, double *output) {
  __m256d out1_1, out2_1, out3_1;
  // process the first chunk (half the input cache line)
  process_chunk_avx2_x1(v_left_1, v_curr_1, v_right_1, v_top_1, v_bot_1, out1_1, out2_1, out3_1, div_x_1, div_y);

  // store one cache line worth of output data
  _mm256_stream_pd(output + 0, out1_1);
  _mm256_stream_pd(output + 4, out2_1);

  __m256d out1_2, out2_2, out3_2;
  // process the second chunk
  process_chunk_avx2_x1(v_left_2, v_curr_2, v_right_2, v_top_2, v_bot_2, out1_2, out2_2, out3_2, div_x_2, div_y);

  // store two cache lines: 1/2 from the previous chunk, 1/2 from this second chunk
  _mm256_stream_pd(output + 8, out3_1);
  _mm256_stream_pd(output + 12, out1_2);
  // and then the rest from the second chunk
  _mm256_stream_pd(output + 16, out2_2);
  _mm256_stream_pd(output + 20, out3_2);
}

// inputs and output must be aligned to 32 bytes
static inline void process_row_avx2_x1(const double *top, const double *current, const double *bottom, double *output,
                                       size_t width, bool first_last_row = false) {
  constexpr size_t ymms_per_iter = 2;  // how many YMM registers worth of input data is processed per loop iteration
  constexpr size_t ymm_size = sizeof(__m256d) / sizeof(double);  // how many `double`s are in a YMM register
  constexpr size_t read_step = ymm_size * ymms_per_iter;         // input width must be a multiple of this

  const __m256d div_c = _mm256_set_pd(0.5, 0.5, 0.5, 0.5);  // to divide dx/dy by 2
  const __m256d div_f = _mm256_set_pd(0.5, 0.5, 0.5, 1.0);  // (dx) do not divide the first element in the row
  const __m256d div_l = _mm256_set_pd(1.0, 0.5, 0.5, 0.5);  // (dx) do not divide the last element in the row
  __m256d div_x_1, div_x_2;                                 // actual divisors for dx are chosen later
  __m256d div_y = (!first_last_row) ? _mm256_set_pd(0.5, 0.5, 0.5, 0.5)
                                    : _mm256_set_pd(1.0, 1.0, 1.0, 1.0);  // (dy) do not divide on the first/last row

  // two YMMs are processed per iteration to fit nicely in a cache line, prefixed by `_1` and `_2`
  __m256d v_left_1, v_curr_1, v_right_1, v_top_1, v_bot_1, v_left_2, v_curr_2, v_right_2, v_top_2, v_bot_2;

  const size_t num_iters = width / (ymm_size * ymms_per_iter);
  for (size_t i = 0; i < num_iters; i++) {
    const size_t read_offset = i * read_step;
    const size_t write_offset = read_offset * 3;  // three values are output per a single input value

    if (i > 0) {                                                    // general case
      v_left_1 = _mm256_load_pd(current + read_offset - ymm_size);  // 4 elements to the left
      div_x_1 = div_c;
    } else {                                                   // this iteration contains the left-most column
      v_left_1 = _mm256_load_pd(current + read_offset);        // take the current 4 elements
      v_left_1 = _mm256_permute4x64_pd(v_left_1, 0b00100100);  // reorder to calculate dx for left-most element
      div_x_1 = div_f;
    }
    v_curr_1 = _mm256_load_pd(current + read_offset);
    v_right_1 = _mm256_load_pd(current + read_offset + ymm_size);
    if (i < num_iters - 1) {                                                    // general case
      v_right_2 = _mm256_load_pd(current + read_offset + ymm_size + ymm_size);  // 8 elements to the right
      div_x_2 = div_c;
    } else {                                                         // this iteration contains the right-most column
      v_right_2 = _mm256_load_pd(current + read_offset + ymm_size);  // take 4 elements to the right
      v_right_2 = _mm256_permute4x64_pd(v_right_2, 0b11100111);      // reorder to calculate dx for right-most element
      div_x_2 = div_l;
    }
    v_top_1 = _mm256_load_pd(top + read_offset);
    v_top_2 = _mm256_load_pd(top + read_offset + ymm_size);
    v_bot_1 = _mm256_load_pd(bottom + read_offset);
    v_bot_2 = _mm256_load_pd(bottom + read_offset + ymm_size);
    v_left_2 = v_curr_1;
    v_curr_2 = v_right_1;

    process_cache_line_avx2_x1(v_left_1, v_curr_1, v_right_1, v_top_1, v_bot_1, v_left_2, v_curr_2, v_right_2, v_top_2,
                               v_bot_2, div_x_1, div_x_2, div_y, output + write_offset);
  }
}

static inline void transpose_avx2(__m256d &row0, __m256d &row1, __m256d &row2, __m256d &row3, __m256d &col0,
                                  __m256d &col1, __m256d &col2, __m256d &col3) {
  __m256d t1 = _mm256_shuffle_pd(row0, row1, 0b0000);
  __m256d t2 = _mm256_shuffle_pd(row0, row1, 0b1111);
  __m256d t3 = _mm256_shuffle_pd(row2, row3, 0b0000);
  __m256d t4 = _mm256_shuffle_pd(row2, row3, 0b1111);

  col0 = _mm256_permute2f128_pd(t1, t3, 0b00100000);
  col2 = _mm256_permute2f128_pd(t1, t3, 0b00110001);
  col1 = _mm256_permute2f128_pd(t2, t4, 0b00100000);
  col3 = _mm256_permute2f128_pd(t2, t4, 0b00110001);
}

static inline void load_row_avx2_x4(const double *input, const size_t channel_stride, const int offset, __m256d &out0,
                                    __m256d &out1, __m256d &out2, __m256d &out3) {
  out0 = _mm256_load_pd(input + channel_stride * 0 + offset);
  out1 = _mm256_load_pd(input + channel_stride * 1 + offset);
  out2 = _mm256_load_pd(input + channel_stride * 2 + offset);
  out3 = _mm256_load_pd(input + channel_stride * 3 + offset);
}

template <int mask>
static inline void permute_doubles_avx2_x4(__m256d &r0, __m256d &r1, __m256d &r2, __m256d &r3) {
  r0 = _mm256_permute4x64_pd(r0, mask);
  r1 = _mm256_permute4x64_pd(r1, mask);
  r2 = _mm256_permute4x64_pd(r2, mask);
  r3 = _mm256_permute4x64_pd(r3, mask);
}

static inline void write_pixelinfo_avx2_x8(__m256d &o0_1, __m256d &o0_2, __m256d &o1_1, __m256d &o1_2, __m256d &o2_1,
                                           __m256d &o2_2, __m256d &o3_1, __m256d &o3_2, const int offset,
                                           double *output) {
  constexpr size_t C = 8;
  constexpr size_t pixel_size = C * 3;
  _mm256_stream_pd(output + pixel_size * 0 + offset + 0, o0_1);
  _mm256_stream_pd(output + pixel_size * 0 + offset + 4, o0_2);
  _mm256_stream_pd(output + pixel_size * 1 + offset + 0, o1_1);
  _mm256_stream_pd(output + pixel_size * 1 + offset + 4, o1_2);
  _mm256_stream_pd(output + pixel_size * 2 + offset + 0, o2_1);
  _mm256_stream_pd(output + pixel_size * 2 + offset + 4, o2_2);
  _mm256_stream_pd(output + pixel_size * 3 + offset + 0, o3_1);
  _mm256_stream_pd(output + pixel_size * 3 + offset + 4, o3_2);
}

static inline void process_row_avx2_x8(const double *top, const double *current, const double *bottom, double *output,
                                       const size_t width, const size_t channel_size, const bool first_last_row = false,
                                       const bool first_col = false, const bool last_col = false) {
  constexpr size_t C = 8;
  constexpr int ymm_size = sizeof(__m256d) / sizeof(double);  // how many `double`s are in a YMM register
  constexpr size_t read_step = ymm_size * 1;                  // input width must be a multiple of this
  constexpr size_t pixel_size = C * 3;                        // aka sizeof(PixelInfo<C>)
  constexpr int reorder_mask_left = 0b00100100;               // permutation mask for dx on the left edge
  constexpr int reorder_mask_right = 0b11100111;              // permutation mask for dx on the right edge

  __m256d i0_1, i0_2, i1_1, i1_2, i2_1, i2_2, i3_1, i3_2;

  const __m256d div_c = _mm256_set_pd(0.5, 0.5, 0.5, 0.5);  // to divide dx/dy by 2
  const __m256d div_f = _mm256_set_pd(0.5, 0.5, 0.5, 1.0);  // (dx) do not divide the first element in the row
  const __m256d div_l = _mm256_set_pd(1.0, 0.5, 0.5, 0.5);  // (dx) do not divide the last element in the row
  const __m256d div_y = (!first_last_row)
                            ? _mm256_set_pd(0.5, 0.5, 0.5, 0.5)
                            : _mm256_set_pd(1.0, 1.0, 1.0, 1.0);  // (dy) do not divide on the first/last row

  const size_t num_iters = width / ymm_size;
  for (size_t i = 0; i < num_iters; i++) {
    const size_t read_offset = i * read_step;
    const size_t write_offset = read_offset * pixel_size;

    // all operations happen in batches of 4 channels at a time instead of 8 to avoid a register spill (since there are
    // only 16 AVX registers), so there are two batches for all three components

    // intensity values
    {
      __m256d c0, c1, c2, c3;
      load_row_avx2_x4(current + read_offset, channel_size, 0, c0, c1, c2, c3);
      transpose_avx2(c0, c1, c2, c3, i0_1, i1_1, i2_1, i3_1);
      load_row_avx2_x4(current + read_offset + channel_size * 4, channel_size, 0, c0, c1, c2, c3);
      transpose_avx2(c0, c1, c2, c3, i0_2, i1_2, i2_2, i3_2);

      write_pixelinfo_avx2_x8(i0_1, i0_2, i1_1, i1_2, i2_1, i2_2, i3_1, i3_2, 0, output + write_offset);
    }

    // dx
    {
      __m256d c0_left, c1_left, c2_left, c3_left;
      __m256d c0_curr, c1_curr, c2_curr, c3_curr;
      __m256d c0_right, c1_right, c2_right, c3_right;
      __m256d dx0, dx1, dx2, dx3;

      const bool has_first_col = first_col && i == 0;
      const bool has_last_col = last_col && i == num_iters - 1;

      const int load_offset_left = !has_first_col ? -ymm_size : 0;
      const int load_offset_right = !has_last_col ? ymm_size : 0;
      const __m256d div_x = !has_first_col && !has_last_col ? div_c : has_first_col ? div_f : div_l;

      load_row_avx2_x4(current + read_offset, channel_size, load_offset_left, c0_left, c1_left, c2_left, c3_left);
      load_row_avx2_x4(current + read_offset, channel_size, 0, c0_curr, c1_curr, c2_curr, c3_curr);
      load_row_avx2_x4(current + read_offset, channel_size, load_offset_right, c0_right, c1_right, c2_right, c3_right);
      if (has_first_col) permute_doubles_avx2_x4<reorder_mask_left>(c0_left, c1_left, c2_left, c3_left);
      if (has_last_col) permute_doubles_avx2_x4<reorder_mask_right>(c0_right, c1_right, c2_right, c3_right);
      dx0 = calculate_dx(c0_left, c0_curr, c0_right, div_x);
      dx1 = calculate_dx(c1_left, c1_curr, c1_right, div_x);
      dx2 = calculate_dx(c2_left, c2_curr, c2_right, div_x);
      dx3 = calculate_dx(c3_left, c3_curr, c3_right, div_x);
      transpose_avx2(dx0, dx1, dx2, dx3, i0_1, i1_1, i2_1, i3_1);

      load_row_avx2_x4(current + read_offset + channel_size * 4, channel_size, load_offset_left, c0_left, c1_left,
                       c2_left, c3_left);
      load_row_avx2_x4(current + read_offset + channel_size * 4, channel_size, 0, c0_curr, c1_curr, c2_curr, c3_curr);
      load_row_avx2_x4(current + read_offset + channel_size * 4, channel_size, load_offset_right, c0_right, c1_right,
                       c2_right, c3_right);
      if (has_first_col) permute_doubles_avx2_x4<reorder_mask_left>(c0_left, c1_left, c2_left, c3_left);
      if (has_last_col) permute_doubles_avx2_x4<reorder_mask_right>(c0_right, c1_right, c2_right, c3_right);
      dx0 = calculate_dx(c0_left, c0_curr, c0_right, div_x);
      dx1 = calculate_dx(c1_left, c1_curr, c1_right, div_x);
      dx2 = calculate_dx(c2_left, c2_curr, c2_right, div_x);
      dx3 = calculate_dx(c3_left, c3_curr, c3_right, div_x);
      transpose_avx2(dx0, dx1, dx2, dx3, i0_2, i1_2, i2_2, i3_2);

      write_pixelinfo_avx2_x8(i0_1, i0_2, i1_1, i1_2, i2_1, i2_2, i3_1, i3_2, 8, output + write_offset);
    }

    // dy
    {
      __m256d c0_top, c1_top, c2_top, c3_top;
      __m256d c0_bot, c1_bot, c2_bot, c3_bot;
      __m256d dy0, dy1, dy2, dy3;
      load_row_avx2_x4(top + read_offset, channel_size, 0, c0_top, c1_top, c2_top, c3_top);
      load_row_avx2_x4(bottom + read_offset, channel_size, 0, c0_bot, c1_bot, c2_bot, c3_bot);
      dy0 = calculate_dy(c0_top, c0_bot, div_y);
      dy1 = calculate_dy(c1_top, c1_bot, div_y);
      dy2 = calculate_dy(c2_top, c2_bot, div_y);
      dy3 = calculate_dy(c3_top, c3_bot, div_y);
      transpose_avx2(dy0, dy1, dy2, dy3, i0_1, i1_1, i2_1, i3_1);

      load_row_avx2_x4(top + read_offset + channel_size * 4, channel_size, 0, c0_top, c1_top, c2_top, c3_top);
      load_row_avx2_x4(bottom + read_offset + channel_size * 4, channel_size, 0, c0_bot, c1_bot, c2_bot, c3_bot);
      dy0 = calculate_dy(c0_top, c0_bot, div_y);
      dy1 = calculate_dy(c1_top, c1_bot, div_y);
      dy2 = calculate_dy(c2_top, c2_bot, div_y);
      dy3 = calculate_dy(c3_top, c3_bot, div_y);
      transpose_avx2(dy0, dy1, dy2, dy3, i0_2, i1_2, i2_2, i3_2);

      write_pixelinfo_avx2_x8(i0_1, i0_2, i1_1, i1_2, i2_1, i2_2, i3_1, i3_2, 16, output + write_offset);
    }
  }
}

void calculate_pixelinfo_avx2_x1(const double *input, double *output, const int width, const int height) {
  for (int y = 0; y < height; y++) {
    const bool is_top_row = (y == 0);
    const bool is_bottom_row = (y == height - 1);

    const double *current_row = input + width * y;
    const double *upper_row = !is_top_row ? input + width * (y - 1) : current_row;
    const double *bottom_row = !is_bottom_row ? input + width * (y + 1) : current_row;

    double *output_row = output + width * y * 3;

    process_row_avx2_x1(upper_row, current_row, bottom_row, output_row, static_cast<size_t>(width),
                        is_top_row || is_bottom_row);
  }
}

void calculate_pixelinfo_avx2_x8(const double *input, double *output, const int width, const int height) {
  // split the image into blocks of this size, so that each block more or less fits into L1/L2 cache
  constexpr int block_size = 128;

  const size_t channel_stride = static_cast<size_t>(width * height);
  for (int y_0 = 0; y_0 < height; y_0 += block_size) {
    for (int x_0 = 0; x_0 < width; x_0 += block_size) {
      for (int y = y_0; y < std::min(y_0 + block_size, height); y++) {
        const int block_width = std::min(block_size, width - x_0);

        const bool is_top_row = y == 0;
        const bool is_bottom_row = y == height - 1;
        const bool has_first_col = x_0 == 0;
        const bool has_last_col = x_0 + block_width == width;

        // pointers are for channel 0, other channels are assumed to be offset by `channel_stride * n`
        const double *current_row_c0 = input + width * y + x_0;
        const double *upper_row_c0 = !is_top_row ? input + width * (y - 1) + x_0 : current_row_c0;
        const double *bottom_row_c0 = !is_bottom_row ? input + width * (y + 1) + x_0 : current_row_c0;

        double *output_row = output + (width * y + x_0) * 3 * 8;

        process_row_avx2_x8(upper_row_c0, current_row_c0, bottom_row_c0, output_row, static_cast<size_t>(block_width),
                            channel_stride, is_top_row || is_bottom_row, has_first_col, has_last_col);
      }
    }
  }
}

#endif  // #ifdef __AVX2__

template <int C, typename T>
static inline void process_row_c(const T *upper_row, const T *current_row, const T *bottom_row, T *output, size_t width,
                                 bool first_last_row = false) {
  for (size_t i = 0; i < width; i++) {
    const size_t write_offset = 3 * C * i;
    output[write_offset] = current_row[i];
    if (i == 0)
      output[write_offset + C] = T(1.0) * (current_row[i + 1] - current_row[i]);
    else if (i == width - 1)
      output[write_offset + C] = T(1.0) * (current_row[i] - current_row[i - 1]);
    else
      output[write_offset + C] = T(0.5) * (current_row[i + 1] - current_row[i - 1]);
    output[write_offset + 2 * C] = ((!first_last_row) ? T(0.5) : T(1.0)) * (bottom_row[i] - upper_row[i]);
  }
}

template <int C, typename T>
void calculate_pixelinfo_c(const T *input, T *output, const int width, const int height) {
  for (int c = 0; c < C; c++) {
    const T *plane_start = input + width * height * c;
    for (int y = 0; y < height; y++) {
      const bool is_top_row = (y == 0);
      const bool is_bottom_row = (y == height - 1);

      const T *current_row = plane_start + width * y;
      const T *upper_row = !is_top_row ? plane_start + width * (y - 1) : current_row;
      const T *bottom_row = !is_bottom_row ? plane_start + width * (y + 1) : current_row;

      T *output_row = output + width * y * 3 * C + c;

      process_row_c<C>(upper_row, current_row, bottom_row, output_row, static_cast<size_t>(width),
                       is_top_row || is_bottom_row);
    }
  }
}

}  // namespace

namespace dsopp::features {

template <int C, typename T>
void calculate_pixelinfo(const T *input, T *output, const int width, const int height) {
  calculate_pixelinfo_c<C>(input, output, width, height);
}

template <>
void calculate_pixelinfo<1>(const double *input, double *output, const int width, const int height) {
#ifdef __AVX2__
  if (width % 8 == 0 && is_aligned(input, 32), is_aligned(output, 32))
    return calculate_pixelinfo_avx2_x1(input, output, width, height);
#endif
  calculate_pixelinfo_c<1>(input, output, width, height);
}

template <>
void calculate_pixelinfo<8>(const double *input, double *output, const int width, const int height) {
#ifdef __AVX2__
  if (width % 4 == 0 && is_aligned(input, 32), is_aligned(output, 32))
    return calculate_pixelinfo_avx2_x8(input, output, width, height);
#endif
  calculate_pixelinfo_c<8>(input, output, width, height);
}

template void calculate_pixelinfo<1>(const float *input, float *output, const int width, const int height);
template void calculate_pixelinfo<8>(const float *input, float *output, const int width, const int height);
template void calculate_pixelinfo<3>(const Precision *input, Precision *output, const int width, const int height);
template void calculate_pixelinfo<16>(const Precision *input, Precision *output, const int width, const int height);
template void calculate_pixelinfo<32>(const Precision *input, Precision *output, const int width, const int height);
template void calculate_pixelinfo<128>(const Precision *input, Precision *output, const int width, const int height);

}  // namespace dsopp::features