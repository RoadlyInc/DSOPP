#!/bin/bash

rm output_small_track.json
out_track="$(pwd)/output_small_track.json"

cd ../../
python3 -m pydsopp.utils.track2json --track ${SMALL_TRACK_BIN} --output $out_track
stat=$?
cd -
if [ "$stat" -ne "0" ]
then
  exit 137
fi

diff ${SMALL_TRACK_JSON} $out_track
stat=$?
rm output_small_track.json
exit $stat
