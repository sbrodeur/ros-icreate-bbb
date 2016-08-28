#!/usr/bin/env bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

INPUT_DATASET_FILE=${DIR}/dataset.bag
OUTPUT_DATASET_FILE=${DIR}/dataset.h5

if [ -f $OUTPUT_DATASET_FILE ]; then
	echo Removing existing dataset file...
	rm $OUTPUT_DATASET_FILE
fi

echo 'Launching conversion...'
python ${DIR}/convert_hdf5.py --input=$INPUT_DATASET_FILE --output=$OUTPUT_DATASET_FILE --start-time=120 --stop-time=795

echo 'All done.'

