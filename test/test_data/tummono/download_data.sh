if [ "$#" -ne 1 ]; then
    echo "sh download_data.sh <sequence_id>"
    exit
fi
SEQ=$(printf "sequence_%02d" $1)
wget https://vision.in.tum.de/mono/dataset/$SEQ.zip
unzip $SEQ.zip
rm $SEQ.zip
unzip $SEQ/images.zip -d $SEQ/images
cp *.yaml $SEQ/
awk 'BEGIN{ RS = "" ; FS = "\n" }{print "tum_fov " $2 " " $1}' $SEQ/camera.txt >> $SEQ/calib.txt
