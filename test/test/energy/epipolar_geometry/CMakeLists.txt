dsopp_add_test(test_generalized_epipolar_line generalized_epipolar_line.cpp
               camera_projector epipolar_geometry opencv settings)

dsopp_add_test_and_coverage(test_triangulation triangulation.cpp
                            epipolar_geometry settings)

dsopp_add_test_and_coverage(test_essential_matrix test_essential_matrix.cpp
                            epipolar_geometry)
