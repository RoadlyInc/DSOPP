import argparse
import numpy as np
from pathlib import Path
import cv2
from tqdm import tqdm
import json
from math import atan, sqrt

from ..track import Track
from ..storage.track_storage import TrackStorage

GLOBAL_R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

def frame_image_name(frame):
    return f'{frame.id}.png'

def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()


def translation_affine_transform(poses):
    positions = []
    R = GLOBAL_R
    for pose in poses:
        positions.append(R @ pose.translation())
    positions = np.array(positions)
    mean = np.mean(positions, axis=0)

    max_distance = np.linalg.norm(R @ poses[0].translation() - mean)
    for pose in poses:
        distance = np.linalg.norm(R @ pose.translation() - mean)
        max_distance = max(float(max_distance), float(distance))

    return max_distance, mean


def sharpness(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fm = variance_of_laplacian(gray)
    return fm


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='NERF format dataset exporter')
    parser.add_argument('--track',
                        dest='track',
                        required=True,
                        help='path to track.bin file')
    parser.add_argument('--output',
                        dest='output',
                        required=True,
                        help='output directory')
    args = parser.parse_args()

    track_storage = TrackStorage()
    track_storage.read(args.track)
    track = Track(track_storage)

    if not track.dumped_images:
        print(f"{args.track} doesn't contain images")
        exit(1)

    image_path = args.output + "/images/"
    Path(image_path).mkdir(parents=True, exist_ok=True)

    calib = track.camera_calibration
    K = calib.model.K
    angle_x = atan(calib.image_size[0] / (K[0, 0] * 2)) * 2
    angle_y = atan(calib.image_size[1] / (K[1, 1] * 2)) * 2
    transforms_json = {
        "camera_angle_x": angle_x,
        "camera_angle_y": angle_y,
        "fl_x": K[0, 0],
        "fl_y": K[1, 1],
        "k1": 0,
        "k2": 0,
        "p1": 0,
        "p2": 0,
        "cx": K[0, 2],
        "cy": K[1, 2],
        "w": calib.image_size[0],
        "h": calib.image_size[1],
        "aabb_scale": 16,
        #"scale": 0.5,
        #"offset": [0.5, 0.5, 0.5],
    }

    frames_json = []

    odometry_poses = [frame.odometry_t_world_frame for frame in track.frames]
    distance, mean = translation_affine_transform(odometry_poses)

    for frame_id, frame in tqdm(enumerate(track.frames),
                                'Processing frames',
                                total=len(track.track.frames)):
        image = frame.image()
        image = cv2.resize(image, (image.shape[1], image.shape[0]))
        file_path = f'images/{frame_image_name(frame)}'
        cv2.imwrite(args.output + '/' + file_path, image)

        pose = odometry_poses[frame_id].matrix()
        
        motion = np.eye(4)
        motion[:3, :3] = GLOBAL_R
        pose = motion @ pose

        pose[:3, 3] = (pose[:3, 3] - mean) / sqrt(2 * distance)

        pose[0:3, 2] *= -1
        pose[0:3, 1] *= -1
        pose = pose[[1, 0, 2, 3], :]
        pose[2, :] *= -1
        

        frame_json = {
            "file_path": file_path,
            "sharpness": sharpness(image),
            "transform_matrix": pose.tolist()
        }
        frames_json.append(frame_json)

    transforms_json["frames"] = frames_json
    json.dump(transforms_json,
              open(args.output + "/transforms.json", 'w'),
              indent=2)
