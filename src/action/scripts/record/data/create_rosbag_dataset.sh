#!/usr/bin/env bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Temporarily setting the internal field seperator (IFS) to the newline character.
IFS=$'\n';

# Recursively loop through all bag files in the specified directory
BAG_DIRECTORY=$1
for bag in $(find ${BAG_DIRECTORY} -name '*.bag.raw'); do
	echo "Processing bag file ${bag}"

	INPUT_DATASET_FILE="${bag}"
	OUTPUT_DATASET_FILE_BAG_PROC_UNBATCHED="${bag}_unbatched.proc"
  OUTPUT_DATASET_FILE_BAG_PROC_MADGWICK="${bag}_madgwick.proc"
  OUTPUT_DATASET_FILE_BAG_PROC_ODOM="${bag}_odom.proc"
	OUTPUT_DATASET_FILE_BAG="${bag%.bag.raw}.bag"

	if [ -f $OUTPUT_DATASET_FILE_BAG ]; then
		echo "Output dataset ${OUTPUT_DATASET_FILE_BAG} already found"
		echo "Skipping conversion for bag file ${INPUT_DATASET_FILE}"
		continue
	fi

	echo "Removing any existing dataset temporary files"
	rm -f $OUTPUT_DATASET_FILE_BAG_PROC_UNBATCHED $OUTPUT_DATASET_FILE_BAG_PROC_MADGWICK $OUTPUT_DATASET_FILE_BAG

	python ${DIR}/convert_rosbag.py --input=$INPUT_DATASET_FILE --output=$OUTPUT_DATASET_FILE_BAG_PROC_UNBATCHED
	if ! [ -f $OUTPUT_DATASET_FILE_BAG_PROC_UNBATCHED ]; then
		echo "Could not find temporary file ${OUTPUT_DATASET_FILE_BAG_PROC_UNBATCHED}. An error probably occured during conversion."
		exit 1
	fi

    rosrun imu_filter_madgwick imu_filter_rosbag -i $OUTPUT_DATASET_FILE_BAG_PROC_UNBATCHED  -o $OUTPUT_DATASET_FILE_BAG_PROC_MADGWICK
    if ! [ -f $OUTPUT_DATASET_FILE_BAG_PROC_MADGWICK ]; then
		echo "Could not find temporary file ${OUTPUT_DATASET_FILE_BAG_PROC_MADGWICK}. An error probably occured during conversion."
		exit 1
	fi

    rosrun create create_odometry_rosbag -i $OUTPUT_DATASET_FILE_BAG_PROC_MADGWICK  -o $OUTPUT_DATASET_FILE_BAG_PROC_ODOM
    if ! [ -f $OUTPUT_DATASET_FILE_BAG_PROC_ODOM ]; then
		echo "Could not find temporary file ${OUTPUT_DATASET_FILE_BAG_PROC_ODOM}. An error probably occured during conversion."
		exit 1
	fi

	cp $OUTPUT_DATASET_FILE_BAG_PROC_ODOM $OUTPUT_DATASET_FILE_BAG

	echo "Compressing rosbag file ${OUTPUT_DATASET_FILE_BAG}"
	rosbag compress --force --bz2 $OUTPUT_DATASET_FILE_BAG
	rm -f ${OUTPUT_DATASET_FILE_BAG%.bag}.orig.bag

	echo "Removing all dataset temporary files"
    rm -f $OUTPUT_DATASET_FILE_BAG_PROC_UNBATCHED $OUTPUT_DATASET_FILE_BAG_PROC_MADGWICK $OUTPUT_DATASET_FILE_BAG_PROC_ODOM

done

echo 'All done.'
