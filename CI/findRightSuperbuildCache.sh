#!/bin/bash

# This script enable to find the cache superbuild for this branch:
# - fisrt: the branch specific superbuild
# - then: a parent/master branch specific superbuild
# - finally: the kitware-master branch superbuild
#
# Let's give you a example to be more specific:
# Suppose my branch name is feature/Slam/.../test/Nick and with try
# to compile under os
# first:
#   ---> os-feature/Slam/.../test/Nick
# then:
#   ---> os-feature/Slam/.../test/master
#   ---> ....
#   ---> ....
#   ---> os-feature/Slam/master
#   ---> os-feature/master
# finally:
#   ---> os-kitware-master

OS=$1
CI_COMMIT_REF=$2
MINIO_CACHE_SERVER_ADDRESS=$3
MINIO_CACHE_SERVER_ACCESS_KEY=$4
MINIO_CACHE_SERVER_SECRET_KEY=$5

prefix=superbuild/runner/minio/project/808

# start timer
START_TIME=$SECONDS

# register cache server
~/mc config host add superbuild ${MINIO_CACHE_SERVER_ADDRESS} ${MINIO_CACHE_SERVER_ACCESS_KEY} ${MINIO_CACHE_SERVER_SECRET_KEY}

# check for the right superbuild
path=${OS}/${CI_COMMIT_REF}
# found the right superbuild
superbuild_cache=$(~/mc find superbuild --name ${path}"-1")
while [ "$superbuild_cache" == "" ]
    do
    echo "check if there exist a specific superbuild for: " $path
    # check if a superbuild exist
    superbuild_cache=$(~/mc find superbuild ${prefix}/${path}/master-1 | grep ${prefix}/${path}/master-1)
    if [ "$path" == "." ]; then
        superbuild_cache=$(~/mc find superbuild ${prefix}/${OS}-kitware-master-1 | grep ${prefix}/${OS}-kitware-master-1)
        if [ "$superbuild_cache" == "" ]; then
            echo "!!!WARNING!!! No default superbuild :" ${OS} "-kitware-master exist, please run a superbuild job for the kitware master or current branch manually via the gitlab interface"
            exit
        fi
    fi
    # get parent dir
    path=`dirname -- ${path}`
done
echo "The cached superbuild: " $superbuild_cache " will be used"

# download and unzip the superbuild
mkdir cache
~/mc cp --recursive $superbuild_cache cache
unzip -q cache/*

echo "Getting the cached superbuild took " $(($SECONDS - $START_TIME)) "s"
