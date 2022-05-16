if(VISUALIZATION)
  add_executable(calculate_average_visualizer_fps
                 calculate_average_visualizer_fps.cpp)
  target_link_libraries(calculate_average_visualizer_fps visualizer sensors
                        sophus test_tools settings)
endif()
