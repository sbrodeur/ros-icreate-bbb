#!/usr/bin/env bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Temporarily setting the internal field seperator (IFS) to the newline character.
IFS=$'\n';

# Recursively loop through all bag files in the specified directory
BAG_DIRECTORY=$1
for bag in $(find ${BAG_DIRECTORY} -name '*.bag'); do 
	echo "Processing bag file ${bag}"
	
	INPUT_DATASET_FILE="${bag}"
	OUTPUT_DATASET_FILE_BAG_PROC="${bag}.proc"
	OUTPUT_DATASET_FILE_BAG="${bag%.bag}.rosbag"
	
	if [ -f $OUTPUT_DATASET_FILE_BAG ]; then
		echo "Output dataset ${OUTPUT_DATASET_FILE_BAG} already found"
		echo "Skipping conversion for bag file ${INPUT_DATASET_FILE}"
		continue
	fi
	
	echo "Removing any existing dataset temporary files"
	rm -f $OUTPUT_DATASET_FILE_BAG_PROC $OUTPUT_DATASET_FILE_BAG
	
	python ${DIR}/convert_rosbag.py --input=$INPUT_DATASET_FILE --output=$OUTPUT_DATASET_FILE_BAG_PROC
	if ! [ -f $OUTPUT_DATASET_FILE_BAG_PROC ]; then
		echo "Could not find temporary file ${OUTPUT_DATASET_FILE_BAG_PROC}. An error probably occured during conversion."
		exit 1
	fi
	cp $OUTPUT_DATASET_FILE_BAG_PROC $OUTPUT_DATASET_FILE_BAG
	
	echo "Compressing rosbag file ${OUTPUT_DATASET_FILE_BAG}"
	rosbag compress --force --bz2 $OUTPUT_DATASET_FILE_BAG
	rm -f ${OUTPUT_DATASET_FILE_BAG%.rosbag}.orig.rosbag
	
	echo "Removing all dataset temporary files"
	rm -f $OUTPUT_DATASET_FILE_BAG_PROC
	
done

echo 'All done.'
