#!/bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

DATASET_DIR=${DIR}/create_dataset
SAMPLE_DATA_BAG=${DIR}/sample.bag
SAMPLE_DATA_H5=${DIR}/sample.h5
SAMPLE_DATA_STATS=${DIR}/sample.stats
SAMPLE_DATA_LOGS=${DIR}/sample.log.csv
SAMPLE_DATA_PREVIEW=${DIR}/sample.preview.mp4

if [ -d "${DATASET_DIR}" ]; then
  	rm -rf ${DATASET_DIR}
fi

# Generate locations and session data for experiment 1
e1Locations=( "C1-3016" "C1-5003" "FLOOR3" )
for i in "${e1Locations[@]}";
do
   echo "Creating new location ${i} for experiment 1"
   locationDir=${DATASET_DIR}/E1/$i
   mkdir -p ${locationDir}
   
   for j in $(seq 1 4);
   do
   		newdate=`date -d "$((RANDOM%1+2016))-$((RANDOM%12+1))-$((RANDOM%28+1)) $((RANDOM%23+1)):$((RANDOM%59+1)):$((RANDOM%59+1))" '+%Y%m%dT%H%M%S'`
   		newsession="${locationDir}/E1_${i/-/}_S${j}_${newdate}"
   		ln -s ${SAMPLE_DATA_BAG} ${newsession}.bag
   		ln -s ${SAMPLE_DATA_H5} ${newsession}.h5
   		ln -s ${SAMPLE_DATA_STATS} ${newsession}.stats
   		ln -s ${SAMPLE_DATA_LOGS} ${newsession}.log.csv
   		ln -s ${SAMPLE_DATA_PREVIEW} ${newsession}.preview.mp4
   		echo "Creating new session data S${j} at location ${i}: ${newsession}"
   done
   
done

# Generate locations and session data for experiment 2
e2Locations=( "CAFETERIA" "FLOOR2" "MEETING" )
for i in "${e2Locations[@]}";
do
   echo "Creating new location ${i} for experiment 2"
   locationDir=${DATASET_DIR}/E2/$i
   mkdir -p ${locationDir}
   
   for j in $(seq 1 8);
   do
   		newdate=`date -d "$((RANDOM%1+2016))-$((RANDOM%12+1))-$((RANDOM%28+1)) $((RANDOM%23+1)):$((RANDOM%59+1)):$((RANDOM%59+1))" '+%Y%m%dT%H%M%S'`
   		newsession="${locationDir}/E2_${i/-/}_S${j}_${newdate}"
   		ln -s ${SAMPLE_DATA_BAG} ${newsession}.bag
   		ln -s ${SAMPLE_DATA_H5} ${newsession}.h5
   		ln -s ${SAMPLE_DATA_STATS} ${newsession}.stats
   		ln -s ${SAMPLE_DATA_LOGS} ${newsession}.log.csv
   		ln -s ${SAMPLE_DATA_PREVIEW} ${newsession}.preview.mp4
   		echo "Creating new session data S${j} at location ${i}: ${newsession}"
   done
   
done

# Generate locations and session data for experiment 3
e3Locations=( "LAB" )
for i in "${e3Locations[@]}";
do
   echo "Creating new location ${i} for experiment 3"
   locationDir=${DATASET_DIR}/E3/$i
   mkdir -p ${locationDir}
   
   for j in $(seq 1 4);
   do
   		newdate=`date -d "$((RANDOM%1+2016))-$((RANDOM%12+1))-$((RANDOM%28+1)) $((RANDOM%23+1)):$((RANDOM%59+1)):$((RANDOM%59+1))" '+%Y%m%dT%H%M%S'`
   		newsession="${locationDir}/E3_${i/-/}_S${j}_${newdate}"
   		ln -s ${SAMPLE_DATA_BAG} ${newsession}.bag
   		ln -s ${SAMPLE_DATA_H5} ${newsession}.h5
   		ln -s ${SAMPLE_DATA_STATS} ${newsession}.stats
   		ln -s ${SAMPLE_DATA_LOGS} ${newsession}.log.csv
   		ln -s ${SAMPLE_DATA_PREVIEW} ${newsession}.preview.mp4
   		echo "Creating new session data S${j} at location ${i}: ${newsession}"
   done
   
done

echo "All done."
