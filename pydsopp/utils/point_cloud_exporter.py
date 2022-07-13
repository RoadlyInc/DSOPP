import numpy as np
import argparse
from laspy import LasHeader
from laspy import ExtraBytesParams
from laspy import LasData

from ..track import Track
from ..storage.track_storage import TrackStorage

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Point cloud exporter')
    parser.add_argument('--track',
                        dest='track',
                        required=True,
                        help='path to track.bin file')
    parser.add_argument('--output',
                        dest='output',
                        required=True,
                        help='output filename')
    parser.add_argument('--coord_system',
                        dest='coord_system',
                        required=True,
                        help='target coordinate system',
                        choices=['ecef', 'odometry'])
    parser.add_argument('--color_scheme',
                        dest='color_scheme',
                        choices=['semantic', 'image_colors'],
                        default='image_colors',
                        help='target color scheme')
    parser.add_argument('--poses_file',
                        dest='poses_file',
                        help='path to file to write poses in')
    parser.add_argument('--file_format',
                        dest='file_format',
                        required=True,
                        choices=['las', 'xyz'],
                        help='output file format')
    parser.add_argument(
        '--las_filtration',
        dest='las_filtration',
        action='store_true',
        help='flag to enable filtration of point cloud in las format')

    args = parser.parse_args()

    track_storage = TrackStorage()
    track_storage.read(args.track)
    track = Track(track_storage)
    min_relative_bs = 0.15
    max_idepth_variance = 5e-8

    if args.coord_system == 'ecef' and not track.valid_localization:
        print(f"Invalid ecef poses inside {args.track}")
        exit(1)

    if args.color_scheme == 'image_colors' and not track.dumped_images:
        print(f"{args.track} doesn't contain images")
        exit(1)

    if args.color_scheme == 'semantic':
        colors = track.get_semantic_colors()
    else:
        colors = track.get_image_colors()

    semantics = track.get_semantics()

    filtration = args.file_format == 'xyz' or (args.file_format == 'las'
                                               and args.las_filtration)

    points_data = []

    if args.coord_system == 'ecef' and args.poses_file is not None:
        with open(args.poses_file, 'w') as p_file:
            for frame in track.frames:
                t_world_frame = frame.ecef_t_world_frame
                t = t_world_frame.translation()
                q = t_world_frame.quaternion()
                q /= q.norm()
                t = f'{t[0]} {t[1]} {t[2]}'
                q = f'{q.b} {q.c} {q.d} {q.a}'
                p_file.write(f'{frame.id} {t} {q}\n')

    for frame, frame_colors, frame_semantics in zip(track.frames, colors,
                                                    semantics):
        if args.coord_system == 'odometry':
            t_world_frame = frame.odometry_t_world_frame
        else:
            t_world_frame = frame.ecef_t_world_frame

        for landmark, color, semantic_id in zip(frame.landmarks, frame_colors,
                                                frame_semantics):
            if filtration and not landmark.confident(min_relative_bs,
                                                     max_idepth_variance):
                continue

            point = t_world_frame @ np.append(landmark.direction,
                                              landmark.idepth)

            if abs(point[3]) < 1e-16:
                continue
            point = point[:3] / point[3]

            point_data = np.array([
                *point, *color, semantic_id, landmark.relative_baseline,
                landmark.idepth_variance
            ])
            points_data.append(point_data)

    if args.file_format == 'xyz':
        save_pc = open(args.output, 'w')
        for point_data in points_data:
            point = point_data[:3]
            color = point_data[3:6].astype(int)
            save_pc.write(
                f'{",".join(map(str, point.tolist() + color.tolist()))}\n')
        save_pc.close()

    elif args.file_format == 'las':
        header = LasHeader(version='1.4', point_format=7)
        header.add_extra_dim(
            ExtraBytesParams(name='relative_baseline', type=np.float64))
        header.add_extra_dim(
            ExtraBytesParams(name='idepth_variance', type=np.float64))
        las = LasData(header=header)

        if args.coord_system == 'ecef':
            EARTH_RADIUS = 6.4e6
            points_data_filtered = [
                point_data for point_data in points_data
                if np.linalg.norm(point_data[:3]) < 1.1 * EARTH_RADIUS
            ]
        else:
            farthest_frame_dist = 0
            for frame in track.frames:
                frame_dist = np.linalg.norm(
                    frame.odometry_t_world_frame.translation())
                farthest_frame_dist = max(frame_dist, farthest_frame_dist)

            points_data_filtered = [
                point_data for point_data in points_data
                if np.linalg.norm(point_data[:3]) < 10 * farthest_frame_dist
            ]

        points_data = np.array(points_data_filtered)
        las.x = points_data[:, 0]
        las.y = points_data[:, 1]
        las.z = points_data[:, 2]
        las.red = points_data[:, 3]
        las.green = points_data[:, 4]
        las.blue = points_data[:, 5]
        las.classification = points_data[:, 6]
        las.relative_baseline = points_data[:, 7]
        las.idepth_variance = points_data[:, 8]
        las.write(args.output)
