import sys

sys.path.insert(0, "${PYDSOPP_DIR}")

IMAGE_PATH = "${TEST_DATA_DIR}/pydsopp/features/image.png"
MIN_FEATURES_NUMBER = 1000

import cv2
from pydsopp.features import extract_tracking_features

image = cv2.imread(IMAGE_PATH)
height, width, _ = image.shape

features = extract_tracking_features(image)

assert len(features) >= MIN_FEATURES_NUMBER

for x, y in features:
    assert 0 <= x <= width
    assert 0 <= y <= height
