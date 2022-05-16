import numpy as np
from tqdm import tqdm
from random import randint

from ..transformations import sim3_from_parameters, se3_from_parameters
from .camera_calibration import CameraCalibration, ModelType
from .frame import Landmark, Frame


def extract_landmarks(landmarks):
    """
        convert proto landmarks to list of ``Landmark``s
    """
    result_landmarks = []

    for landmark in landmarks:
        projection = np.array([landmark.projection_x, landmark.projection_y])
        direction = np.array(
            [landmark.direction_x, landmark.direction_y, landmark.direction_z])

        result_landmarks.append(
            Landmark(projection, direction, landmark.idepth,
                     landmark.idepth_variance, landmark.relative_baseline,
                     landmark.semantic_type_id))

    return result_landmarks


def extract_landmarks(landmarks):
    """
        convert proto landmarks to list of ``Landmark``s
    """
    result_landmarks = []

    for landmark in landmarks:
        projection = np.array([landmark.projection_x, landmark.projection_y])
        direction = np.array(
            [landmark.direction_x, landmark.direction_y, landmark.direction_z])

        result_landmarks.append(
            Landmark(projection, direction, landmark.idepth,
                     landmark.idepth_variance, landmark.relative_baseline,
                     landmark.semantic_type_id))

    return result_landmarks


class Track:
    """
    Class for exctract data from track storage
    """

    def __init__(self, track_storage):
        """
        Arguments:
            track_storage python track storage
        """
        self.track = track_storage
        self.main_sensor_id = 0

        self.valid_localization = len(
            self.track.ecef_poses.t_local_keyframes) == len(self.track.frames)

        self.dumped_images = (len(
            self.track.frames[0].image_buffer[self.main_sensor_id]) != 0)

        image_size = self.track.agent_settings.camera_settings[
            self.main_sensor_id].image_size
        intrinsics = self.track.agent_settings.camera_settings[
            self.main_sensor_id].intrinsics
        model_type = ModelType(self.track.agent_settings.camera_settings[
            self.main_sensor_id].model_type)
        self.camera_calibration = CameraCalibration(intrinsics, image_size,
                                                    model_type)

        self.t_earth_local = sim3_from_parameters(
            self.track.ecef_poses.t_earth_local)

        self.ecef_poses = list(
            map(lambda data: self.t_earth_local @ sim3_from_parameters(data), [
                pose.data for pose in self.track.ecef_poses.t_local_keyframes
            ]))

        self.odometry_poses = list(
            map(se3_from_parameters,
                [frame.t_world_agent for frame in self.track.frames]))

        self.frames = []
        for frame_id, frame in tqdm(enumerate(self.track.frames),
                                    'Extract frames',
                                    total=len(self.track.frames)):
            landmarks = frame.landmarks[self.main_sensor_id].landmarks
            landmarks = extract_landmarks(landmarks)

            odometry_pose = self.odometry_poses[frame_id]

            ecef_pose = None if not self.valid_localization else self.ecef_poses[
                frame_id]

            attached_frames = []
            attached_frame_id = frame.id
            for attached_frame in frame.tracking_frames:
                attached_frame_id += 1
                attached_frames.append(
                    Frame.AttachedFrame(
                        attached_frame_id, attached_frame.timestamp,
                        se3_from_parameters(attached_frame.t_keyframe_agent)))

            self.frames.append(
                Frame(frame.id, frame.timestamp, odometry_pose, ecef_pose,
                      landmarks, frame.image_buffer[self.main_sensor_id],
                      attached_frames))

    def get_image_colors(self):
        """
        Extracts colors (RGB) for all landmarks from dumped images
        returns None if there are no images
        """
        if not self.dumped_images:
            return None

        colors = []
        for frame_id, frame in tqdm(enumerate(self.frames),
                                    "Extracting colors",
                                    total=len(self.frames)):
            image = frame.image()
            frame_colors = []
            for landmark in frame.landmarks:
                color = image[int(landmark.projection[1]),
                              int(landmark.projection[0])]
                frame_colors.append(color.tolist()[::-1])
            colors.append(frame_colors)

        return colors

    def get_semantic_colors(self):
        """
        Create random colors for points by semantic types
        Returns list of colors
        """
        type_color_map = {}
        colors = []

        for frame_id, frame in enumerate(self.frames):
            frame_colors = []
            for landmark in frame.landmarks:
                semantic_type_id = landmark.semantic_type_id
                if semantic_type_id not in type_color_map:
                    type_color_map[semantic_type_id] = [
                        randint(0, 255) for i in range(3)
                    ]
                frame_colors.append(type_color_map[semantic_type_id])
            colors.append(frame_colors)

        return colors

    def get_semantics(self):
        """
        Returns semantic type ids
        """
        semantics = []
        for frame_id, frame in enumerate(self.frames):
            frame_semantics = []
            for landmark in frame.landmarks:
                semantic_type_id = landmark.semantic_type_id
                frame_semantics.append(semantic_type_id)
            semantics.append(frame_semantics)

        return semantics

    def get_connections(self):
        """
        Returns connections between frames
        """
        connections = {}
        for connection in self.track.connections.connections:
            reference_id = connection.reference_keyframe_id
            target_id = connection.target_keyframe_id
            if reference_id not in connections:
                connections[reference_id] = []
            connections[reference_id].append(target_id)

        return connections
