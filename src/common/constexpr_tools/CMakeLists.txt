add_library(common_constexpr_tools INTERFACE)

target_include_directories(
  common_constexpr_tools
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/dsopp>)
