#!/bin/bash

./save_random_track && \
python3 read_and_save.py && \
./compare_two_tracks && \
rm "track.bin" "track_python.bin"

