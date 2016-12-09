#!/usr/bin/env bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Temporarily setting the internal field seperator (IFS) to the newline character.
IFS=$'\n';

# Recursively loop through all bag files in the specified directory
BAG_DIRECTORY=$1
for bag in $(find ${BAG_DIRECTORY} -name '*.bag'); do
	echo "Processing bag file ${bag}"

	INPUT_DATASET_FILE="${bag}"
	OUTPUT_STATS_FILE="${bag%.bag}.stats"

	if [ -f $OUTPUT_STATS_FILE ]; then
		echo "Output statistics file ${OUTPUT_STATS_FILE} already found"
		echo "Skipping analysis for bag file ${INPUT_DATASET_FILE}"
		continue
	fi

	python ${DIR}/stats_rosbag.py --input=$INPUT_DATASET_FILE --output=$OUTPUT_STATS_FILE --ignore-topics="/rosout,/rosout_agg,/tf,/irobot_create/cmd_raw" --drop-threshold=1.0 
	if ! [ -f $OUTPUT_STATS_FILE ]; then
		echo "Could not find output file ${OUTPUT_STATS_FILE}. An error probably occured during analysis."
		exit 1
	fi

done

echo 'All done.'
