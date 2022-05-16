from tqdm import tqdm
import numpy as np
import json


def static_frame_info(frame):
    if frame.ecef_t_frame_world is None:
        matrix = frame.odometry_t_world_frame.matrix()
    else:
        matrix = frame.ecef_t_world_frame.matrix()
    pose_array = np.concatenate([*matrix[:3]]).tolist()

    return {
        "time": float(frame.timestamp) / 1e+9,
        "id": frame.id,
        "camera": pose_array
    }


def json_track_export(track,
                      json_filepath,
                      min_relative_bs=0.2,
                      max_idepth_variance=1e-10):
    if track.dumped_images:
        colors = track.get_image_colors()
    else:
        colors = track.get_semantic_colors()

    frames_json = []

    for frame, frame_color in tqdm(zip(track.frames, colors),
                                   "Jsoning frames",
                                   total=len(colors)):
        frame_json = static_frame_info(frame)

        cloud = []
        cloud_color = []
        for landmark, color in zip(frame.landmarks, frame_color):

            if not landmark.confident(min_relative_bs, max_idepth_variance):
                continue

            point = landmark.direction / landmark.idepth
            cloud += point.tolist()
            cloud_color += color

        frame_json["cloud"] = cloud
        frame_json["colors"] = cloud_color
        frames_json.append((frame_json))

    camera_json = track.camera_calibration.json()
    resulting_json = camera_json.copy()
    resulting_json["frames"] = frames_json
    json.dump(resulting_json, open(json_filepath, 'w'))
