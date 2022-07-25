import sys, os

STORAGE_PATH = os.path.dirname(__file__)
sys.path.insert(
    0, os.path.abspath(os.path.join(STORAGE_PATH,
                                    "../../../pydsopp/storage/")))

from track_storage import TrackStorage

track_storage = TrackStorage()
track_storage.read(os.path.join(STORAGE_PATH, "track.bin"))
track_storage.save(os.path.join(STORAGE_PATH, "track_python.bin"))
