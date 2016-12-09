#!/usr/bin/env bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Temporarily setting the internal field seperator (IFS) to the newline character.
IFS=$'\n';

DATASET_DIRECTORY=$1

bash $DIR/create_rosbag_dataset.sh $DATASET_DIRECTORY
if ! [ $? -eq 0 ]; then
  echo "Error calling script $DIR/create_rosbag_dataset.sh $DATASET_DIRECTORY"
  exit 1
fi

bash $DIR/create_rosbag_logs.sh $DATASET_DIRECTORY
if ! [ $? -eq 0 ]; then
  echo "Error calling script $DIR/create_rosbag_logs.sh $DATASET_DIRECTORY"
  exit 1
fi

bash $DIR/create_rosbag_stats_crop.sh $DATASET_DIRECTORY $DATASET_DIRECTORY
if ! [ $? -eq 0 ]; then
  echo "Error calling script $DIR/create_rosbag_cropped.sh $DATASET_DIRECTORY"
  exit 1
fi

bash $DIR/create_rosbag_stats.sh $DATASET_DIRECTORY
if ! [ $? -eq 0 ]; then
  echo "Error calling script $DIR/create_rosbag_stats.sh $DATASET_DIRECTORY"
  exit 1
fi

bash $DIR/create_hdf5_dataset.sh $DATASET_DIRECTORY
if ! [ $? -eq 0 ]; then
  echo "Error calling script $DIR/create_hdf5_dataset.sh $DATASET_DIRECTORY"
  exit 1
fi

bash $DIR/create_hdf5_visualization.sh $DATASET_DIRECTORY
if ! [ $? -eq 0 ]; then
  echo "Error calling script $DIR/create_hdf5_visualization.sh $DATASET_DIRECTORY"
  exit 1
fi

echo 'All done.'
