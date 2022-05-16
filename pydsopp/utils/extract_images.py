import cv2
from tqdm import tqdm
import argparse
from pathlib import Path

from ..track.track import Track
from ..storage.track_storage import TrackStorage

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='control saving data')
    parser.add_argument('--images',
                        metavar='-pc',
                        dest='images',
                        required=True,
                        help='path to the folder for saving images')

    parser.add_argument('--track',
                        metavar='-t',
                        dest='track',
                        required=True,
                        help='absolute path to track.bin file')

    args = parser.parse_args()

    track_storage = TrackStorage()
    track_storage.read(args.track)
    track = Track(track_storage)

    if not track.dumped_images:
        print(f"{args.track} doesn't contain images")
        exit(1)

    path = args.images
    Path(path).mkdir(parents=True, exist_ok=True)

    for frame_id, frame in tqdm(enumerate(track.frames),
                                'Dumping keyframes',
                                total=len(track.track.frames)):
        cv2.imwrite(f'{path}/{frame_id}.jpg', frame.image())
