dsopp_add_test(test_reprojects test_reprojects.cpp camera_projector ceres glog
               settings)

dsopp_add_test_and_coverage(test_projector test_projector.cpp ceres glog
                            camera_projector settings)
