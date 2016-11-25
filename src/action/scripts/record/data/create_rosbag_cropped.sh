#!/usr/bin/env bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Temporarily setting the internal field seperator (IFS) to the newline character.
IFS=$'\n';

# Recursively loop through all bag files in the specified directory
BAG_DIRECTORY=$1
for bag in $(find ${BAG_DIRECTORY} -name '*.bag'); do
	echo "Processing bag file ${bag}"

	INPUT_DATASET_FILE="${bag}"
	OUTPUT_DATASET_FILE_BAG_PROC_CROPPED="${bag}_cropped.proc"
  OUTPUT_DATASET_FILE_BAG="${bag%.bag.raw}.bag"
  OUTPUT_DROPPED_HISTOGRAM="${bag%.bag}.droppedMsgs"

  echo "Removing any existing dataset temporary files"
	rm -f $OUTPUT_DATASET_FILE_BAG_PROC_CROPPED


  python ${DIR}/extract_rosbag.py --input=$INPUT_DATASET_FILE --output=$OUTPUT_DATASET_FILE_BAG_PROC_CROPPED --ignore-topics="/rosout,/rosout_agg,/tf,/irobot_create/cmd_raw" --drop-threshold=1.0 --extract=600 --windowConv=10 --cropWindow=15 --saveGraph=$OUTPUT_DROPPED_HISTOGRAM
	if ! [ -f $OUTPUT_DATASET_FILE_BAG_PROC_CROPPED ]; then
		echo "Could not find output file ${OUTPUT_DATASET_FILE_BAG_PROC_CROPPED}. An error probably occured during cropping."
		exit 1
	fi

  cp $OUTPUT_DATASET_FILE_BAG_PROC_CROPPED $INPUT_DATASET_FILE

	echo "Removing all dataset temporary files"
    rm -f $OUTPUT_DATASET_FILE_BAG_PROC_CROPPED


done

echo 'All done.'
