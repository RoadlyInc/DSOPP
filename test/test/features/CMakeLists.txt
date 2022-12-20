dsopp_add_test(
  test_tracking_features_extractor test_tracking_features_extractor.cpp
  features sensors camera_providers camera_calibration)

dsopp_add_test(test_dxdy_accelerated test_dxdy_accelerated.cpp features
               settings)
