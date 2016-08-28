#!/bin/bash

# Information
USERNAME="root"

# Project directory on local computer
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
PROJECT_ROOT=$( cd "$( dirname "${DIR}/../.." )" && pwd )

# Sync root directory
echo "Syncing Catkin workspace ${PROJECT_ROOT} to Beaglebone ..."
cd ${PROJECT_ROOT}
echo "build" >> ${DIR}/.exclude
echo "devel" >> ${DIR}/.exclude
echo ".svn" >> ${DIR}/.exclude
echo ".exclude" >> ${DIR}/.exclude

OPT_ARGS=''
if [ "$1" = "reset" ]; then
   echo 'WARNING: any unknown file in the target directory will be deleted.'
   read -p "Are you sure? (y/n)" -n 1 -r
   echo    # (optional) move to a new line
   if [[ $REPLY =~ ^[Yy]$ ]]; then
		OPT_ARGS='--delete'
   fi
fi

rsync -zre ssh --chmod=u+rw --times --exclude-from ${DIR}/.exclude --recursive --update ${OPT_ARGS} --progress --human-readable ${PROJECT_ROOT} ${USERNAME}@192.168.7.2:/root/work
echo 'done.'

