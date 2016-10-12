#!/usr/bin/env bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Temporarily setting the internal field seperator (IFS) to the newline character.
IFS=$'\n';

# Recursively loop through all bag files in the specified directory
DATASET_DIRECTORY=$1
for data in $(find ${DATASET_DIRECTORY} -name '*.h5'); do
	echo "Processing dataset file ${data}"
	
	INPUT_DATASET_FILE="${data}"
	OUTPUT_VISUALIZATION_FILE="${data%.h5}.mp4"
	OUTPUT_VISUALIZATION_PREVIEW_FILE="${data%.h5}.preview.mp4"
	
	if [ -f $OUTPUT_VISUALIZATION_FILE ]; then
		echo "Output dataset ${OUTPUT_VISUALIZATION_FILE} already found"
		echo "Skipping conversion for dataset file ${INPUT_DATASET_FILE}"
		continue
	fi
	
	# Create videos for all sensors
	TMPDIR=`mktemp -d`
	echo "Using temporary directory ${TMPDIR}"
	python ${DIR}/visualize_hdf5.py --input=$INPUT_DATASET_FILE --output-dir=$TMPDIR --nb-processes=-1 --downsample-ratio=4
	
	# Tile all videos into a mosaic and add stereo audio
	ffmpeg \
	-i $TMPDIR/video_left.avi -i $TMPDIR/video_right.avi -i $TMPDIR/imu_linear_acceleration.avi -i $TMPDIR/imu_angular_velocity.avi -i $TMPDIR/imu_magnetic_field.avi -i $TMPDIR/imu_orientation.avi -i $TMPDIR/audio_left-right.wav \
	-filter_complex "
		nullsrc=size=1280x720 [base];
		[0:v] setpts=PTS-STARTPTS, scale=640x480 [upperleft];
		[1:v] setpts=PTS-STARTPTS, scale=640x480 [upperright];
		[2:v] setpts=PTS-STARTPTS, scale=320x240 [lowerleft];
		[3:v] setpts=PTS-STARTPTS, scale=320x240 [lowerleftcenter];
		[4:v] setpts=PTS-STARTPTS, scale=320x240 [lowerrightcenter];
		[5:v] setpts=PTS-STARTPTS, scale=320x240 [lowerright];
		[base][upperleft] overlay=shortest=1 [tmp1];
		[tmp1][upperright] overlay=shortest=1:x=640:y=0 [tmp2];
		[tmp2][lowerleft] overlay=shortest=1:x=0:y=480 [tmp3];
		[tmp3][lowerleftcenter] overlay=shortest=1:x=320:y=480 [tmp4];
		[tmp4][lowerrightcenter] overlay=shortest=1:x=640:y=480 [tmp5];
		[tmp5][lowerright] overlay=shortest=1:x=960:y=480
	" \
	-c:v libx264 -r 20 -c:a libvorbis -shortest $OUTPUT_VISUALIZATION_FILE
	
	if ! [ -f $OUTPUT_VISUALIZATION_FILE ]; then
		echo "Could not find output file ${OUTPUT_VISUALIZATION_FILE}. An error probably occured during conversion."
		echo "Removing all visualization temporary files"
		rm -rf $TMPDIR
		exit 1
	fi
	
	# Create a fast-forward (10x) preview video without audio
	ffmpeg -i $OUTPUT_VISUALIZATION_FILE -vf "setpts=(1/10)*PTS" -an $OUTPUT_VISUALIZATION_PREVIEW_FILE
	
	echo "Removing all visualization temporary files"
	rm -rf $TMPDIR
	
done

echo 'All done.'

