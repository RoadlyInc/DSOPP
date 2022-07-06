#ifndef DSOPP_BUFFER_STORAGE_HPP
#define DSOPP_BUFFER_STORAGE_HPP

#include <map>

#include <GL/glew.h>
#include <pangolin/pangolin.h>

namespace dsopp {
namespace output {
/** GL buffers of points with colors from the frame for the visualization */
struct BufferStorage {
  /** vertex buffer for drawing the frame */
  std::map<size_t, pangolin::GlBuffer> points;
  /** color buffer for drawing the frame */
  std::map<size_t, pangolin::GlBuffer> colors;
  /** default color buffer for drawing the frame */
  std::map<size_t, pangolin::GlBuffer> default_colors;
  /** Track colors, unique for each track */
  std::map<size_t, pangolin::GlBuffer> track_specific_color;
  /** seamntic colots for drawing the frame */
  std::map<size_t, pangolin::GlBuffer> semantic_colors;
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_BUFFER_STORAGE_HPP
