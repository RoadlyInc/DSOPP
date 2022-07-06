from tqdm import tqdm
import numpy as np
import cv2
from pathlib import Path
from scipy.spatial import cKDTree
from dataclasses import dataclass

from ..camera_calibration import Pinhole, CameraCalibration


class Point3D:
    """
    Internal class for building colmap data, representation of 3D point with index and 2D projection indices
    """
    @dataclass
    class Projection:
        frame_index: int
        point_index: int

    def __init__(self, position, index, color):
        self.position = position
        self.index = index
        self.projections = []
        self.color = color

    def p_str(self):
        return " ".join(map(str, self.position))


class Frame:
    """
    Internal class for building colmap data
    """
    @dataclass
    class Point2D:
        pixel: np.array
        index: int
        point3d_index: int

    def __init__(self, index, t_world_frame, image_name):
        t_frame_world = t_world_frame.se3().inverse()
        self.index = index
        self.quaternion = t_frame_world.quaternion()
        self.translation = t_frame_world.translation()
        self.image_name = image_name
        self.points2d = []

        self.depth_map_resolution = 5  #pixels
        self.depth_map = {}

    def push_point(self, pixel, point3d, point3d_index):
        key = tuple((pixel / self.depth_map_resolution).astype(int))

        point3d_dist = np.linalg.norm(point3d)

        if key not in self.depth_map:
            self.depth_map[key] = (pixel, point3d_dist, point3d_index)
            return

        if self.depth_map[key][1] > point3d_dist:
            self.depth_map[key] = (pixel, point3d_dist, point3d_index)

    def finalize(self):
        """
        Populate ``self.points2d`` array with projected onto ``self.depth_map`` points
        """
        for pixel, point3d_dist, point3d_index in self.depth_map.values():
            point2d_idx = len(self.points2d)
            self.points2d.append(
                Frame.Point2D(pixel, point2d_idx, point3d_index))

    def q_str(self):
        return f"{self.quaternion.a} {self.quaternion.b} {self.quaternion.c} {self.quaternion.d}"

    def t_str(self):
        return " ".join(map(str, self.translation))


def cameras_txt(tracks, filename):
    with open(filename, 'w') as f:
        for i, track in enumerate(tracks):
            camera_str = ''
            if isinstance(track.camera_calibration.model, Pinhole):
                camera_str = f"{i+1} PINHOLE {str(track.camera_calibration)}\n"
            else:
                assert False, f"{type(track.camera_calibration.model)} is not supported"
            f.write(camera_str)


def image_name(track_i, frame):
    return f"{track_i}_{frame.id}.png"


def dump_images(track_i, track, image_path):
    for frame in tqdm(track.frames, "Dumping keyframe images"):
        cv2.imwrite(f"{image_path}/{image_name(track_i, frame)}",
                    frame.image())


def images_txt(frames, filename):
    with open(filename, 'w') as f:
        for frame in frames:
            f.write(
                f"{frame.index+1} {frame.q_str()} {frame.t_str()} 1 {frame.image_name}\n"
            )
            point2d_str = []
            for point2d in frame.points2d:
                point2d_str.append(
                    f"{point2d.pixel[0]} {point2d.pixel[1]} {point2d.point3d_index + 1}"
                )

            f.write(" ".join(point2d_str) + "\n")


def points3D_txt(points, filename):
    with open(filename, 'w') as f:
        for point in points:
            head = f"{point.index + 1} {point.p_str()} {point.color[0]} {point.color[1]} {point.color[2]} 0 "

            track_str = []
            for p in point.projections:
                track_str.append(f"{p.frame_index + 1} {p.point_index + 1}")

            if len(point.projections) != 0:
                f.write(head + " ".join(track_str) + "\n")


def build_by_pointcloud(tracks, min_relative_bs, max_idepth_variance):
    pointcloud = []
    cloud_pts = []

    print("Cloud transformation :")
    print("Quaternion: ", tracks[0].t_earth_local.quaternion())
    print("Translation :", tracks[0].t_earth_local.translation())

    for track_i, track in enumerate(tracks):
        colors = track.get_image_colors()
        for frame, frame_colors in tqdm(zip(track.frames, colors),
                                        "Extracting cloud",
                                        total=len(track.frames)):
            t_world_frame = track.t_earth_local.inverse(
            ) @ frame.ecef_t_world_frame
            for landmark, color in zip(frame.landmarks, frame_colors):
                if not landmark.confident(min_relative_bs,
                                          max_idepth_variance):
                    continue

                coords = landmark.direction / landmark.idepth
                coords = t_world_frame @ coords

                point3d = Point3D(coords, len(pointcloud), color)

                cloud_pts.append(coords)
                pointcloud.append(point3d)

    pointcloud_tree = cKDTree(cloud_pts)

    search_radius = 25
    colmap_frames = []
    frame_id = 0
    for track_i, track in enumerate(tracks):
        for frame in tqdm(track.frames,
                          "Projecting points",
                          total=len(track.frames)):
            t_world_frame = track.t_earth_local.inverse(
            ) @ frame.ecef_t_world_frame
            t_frame_world = t_world_frame.inverse()

            nearest_point_ids = pointcloud_tree.query_ball_point(
                t_world_frame.translation(), search_radius)

            colmap_frame = Frame(frame_id, t_world_frame,
                                 f"images/{image_name(track_i, frame)}")
            frame_id += 1
            for point_id in nearest_point_ids:

                point_in_frame = t_frame_world @ cloud_pts[point_id]
                point_in_image = track.camera_calibration.project(
                    point_in_frame)

                if point_in_image is not None:
                    colmap_frame.push_point(point_in_image, point_in_frame,
                                            pointcloud[point_id].index)
            colmap_frame.finalize()

            for point2d in colmap_frame.points2d:

                pointcloud[point2d.point3d_index].projections.append(
                    Point3D.Projection(colmap_frame.index, point2d.index))

            colmap_frames.append(colmap_frame)
    return colmap_frames, pointcloud


def colmap_track_export(tracks,
                        sparse_reconstruction_path,
                        min_relative_bs=0.15,
                        max_idepth_variance=1e-8):
    """
    Arguments:
        tracks -- parsed ``track.bin`` files
        sparse_reconstruction_path -- path to save `images.txt`, `cameras.txt`, `points3D.txt` and `images/`
        min_relative_bs -- minimal relative baseline threshold (if greater then accept)
        max_idepth_variance -- max idepth variance threshold (if less then accept)
    """
    for track in tracks:
        if not track.valid_localization:
            print("Invalid ecef poses in one of tracks")
            exit(0)

    Path(sparse_reconstruction_path).mkdir(parents=True, exist_ok=True)
    print("Dumping cameras.txt")
    cameras_txt(tracks, f"{sparse_reconstruction_path}/cameras.txt")

    for track_i, track in enumerate(tracks):
        if track.dumped_images:
            print("Dumping images")
            img_path = f"{sparse_reconstruction_path}/images/"
            Path(img_path).mkdir(parents=True, exist_ok=True)
            dump_images(track_i, track, img_path)
        else:
            print(f"Images were not saved in track.bin for {track_i}")

    colmap_frames, pointcloud = build_by_pointcloud(tracks, min_relative_bs,
                                                    max_idepth_variance)

    print("Dumping images.txt")
    images_txt(colmap_frames, f"{sparse_reconstruction_path}/images.txt")

    print("Dumping points3D.txt")
    points3D_txt(pointcloud, f"{sparse_reconstruction_path}/points3D.txt")
