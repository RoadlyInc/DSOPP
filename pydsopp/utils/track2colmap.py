import argparse

from ..track import Track, colmap_track_export
from ..storage.track_storage import TrackStorage


def read_track(track_path):
    track_storage = TrackStorage()
    track_storage.read(track_path)
    return Track(track_storage)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Track to colmap .txt format exporter')
    parser.add_argument(
        '--track',
        dest='track',
        required=True,
        help=
        'path to track.bin files (multiple). Multiple tracks option is available only for tracks with same T_world_local pose e.g. after relocalization module',
        nargs='+')
    parser.add_argument('--output',
                        dest='output',
                        required=True,
                        help='output directory path')
    parser.add_argument('--pose_mode',
                        dest='pose_mode',
                        default='odometry',
                        help='Output poses `ecef` or `odometry`')

    args = parser.parse_args()

    tracks = [read_track(track_path) for track_path in args.track]
    colmap_track_export(tracks, args.output, args.pose_mode)
