#!/usr/bin/env bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Temporarily setting the internal field seperator (IFS) to the newline character.
IFS=$'\n';

# Recursively loop through all bag files in the specified directory
BAG_DIRECTORY=$1
for bag in $(find ${BAG_DIRECTORY} -name '*.bag'); do 
	echo "Processing bag file ${bag}"
	
	INPUT_DATASET_FILE="${bag}"
	OUTPUT_LOG_FILE="${bag%.bag}.log.csv"
	
	if [ -f $OUTPUT_LOG_FILE ]; then
		echo "Output log file ${OUTPUT_LOG_FILE} already found"
		echo "Skipping analysis for bag file ${INPUT_DATASET_FILE}"
		continue
	fi
	
	python ${DIR}/logs_rosbag.py --input=$INPUT_DATASET_FILE --output=$OUTPUT_LOG_FILE
	if ! [ -f $OUTPUT_STATS_FILE ]; then
		echo "Could not find output file ${OUTPUT_LOG_FILE}. An error probably occured during analysis."
		exit 1
	fi
	
done

echo 'All done.'

