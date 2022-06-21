import numpy as np
import cv2
from dataclasses import dataclass


@dataclass
class Landmark:
    """
    landmark storage
    """
    projection: np.array
    direction: np.array
    idepth: float
    idepth_variance: float
    relative_baseline: float
    semantic_type_id: int

    def confident(self, min_relative_bs, max_idepth_variance):
        return self.idepth_variance < max_idepth_variance and self.relative_baseline > min_relative_bs


class Frame:
    """
    Frame in the world
    """

    class AttachedFrame:
        """
        Frame attached to a keyframe (Frame)
        """

        def __init__(self, id_, timestamp, odometry_t_keyframe_frame):
            self.id = id_
            self.timestamp = timestamp
            self.odometry_t_keyframe_frame = odometry_t_keyframe_frame

    def __init__(self, id_, timestamp, odometry_pose, ecef_pose, landmarks,
                 image_buffer, attached_frames):
        self.id = id_
        self.timestamp = timestamp

        self.odometry_t_world_frame = odometry_pose
        self.ecef_t_world_frame = ecef_pose
        self.odometry_t_frame_world = odometry_pose.inverse()

        if ecef_pose is not None:
            self.ecef_t_frame_world = ecef_pose.inverse()
        else:
            self.ecef_t_frame_world = None

        self.landmarks = landmarks
        self.image_buffer = image_buffer
        self.attached_frames = attached_frames

    def image(self):
        return cv2.imdecode(
            np.asarray(bytearray(self.image_buffer), dtype=np.uint8),
            cv2.IMREAD_UNCHANGED)
