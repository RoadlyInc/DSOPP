import argparse

from ..track import Track, json_track_export
from ..storage.track_storage import TrackStorage

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Track to json exporter')
    parser.add_argument('--track',
                        dest='track',
                        required=True,
                        help='path to track.bin file')
    parser.add_argument('--output',
                        dest='output',
                        required=True,
                        help='output json filename')
    args = parser.parse_args()

    track_storage = TrackStorage()
    track_storage.read(args.track)
    track = Track(track_storage)

    json_track_export(track, args.output)
