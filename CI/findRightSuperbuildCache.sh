#! /bin/bash

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

# not protecting against spaces in OS name or commit name,
# because that is something that is very wrong and should not be
OS=$1
CI_COMMIT_REF=$2
MINIO_CACHE_SERVER_ADDRESS=$3
MINIO_CACHE_SERVER_ACCESS_KEY=$4
MINIO_CACHE_SERVER_SECRET_KEY=$5

echo "inside findRightSuperbuildCache, OS is: $OS"
if [ $OS == "windows_10" ]; then
    MC=/cygdrive/c/mc.exe
else
    MC=~/mc
fi

prefix=superbuild/runner/minio/project/808

# start timer
START_TIME=$SECONDS

# register cache server
$MC config host add superbuild ${MINIO_CACHE_SERVER_ADDRESS} ${MINIO_CACHE_SERVER_ACCESS_KEY} ${MINIO_CACHE_SERVER_SECRET_KEY}

path="${OS}/${CI_COMMIT_REF}"

echo "The builboot is now trying to find a superbuild on the server for this specific branch."

function cache_exist {
    if echo $($MC find superbuild "$1" 2>&1) | grep -i Error; then
        return 1 # does not exists, return false
    else
	return 0
    fi
}

# find the most appropriate superbuild
superbuild_cache="${prefix}/${path}-1"
echo "looking for superbuild cache at: $superbuild_cache"
while ! cache_exist "$superbuild_cache"; do
    echo "cache not found"
    path=$(dirname -- ${path})
    if [ "$path" != "." ]; then
	superbuild_cache="${prefix}/${path}/master-1"
	echo "looking for superbuild cache at: $superbuild_cache"
    else
        superbuild_cache="${prefix}/${OS}/kitware-master-1"
	echo "top level reached, looking for superbuild cache at: ${superbuild_cache}"
        if ! cache_exist "$superbuild_cache"; then
            echo "!!!WARNING!!! Default superbuild : ${superbuild_cache} does not exists"
	    echo "please run a superbuild job for the kitware master or current branch manually via the gitlab interface"
            exit 1
        fi
    fi
done

echo "Using superbuild cache: ${superbuild_cache}"
echo "Downloading and uncompressing the Superbuild. This take some time..."
# download and unzip the superbuild
mkdir cache
$MC cp --recursive "$superbuild_cache" cache

tar xf cache/*

rm cache/*

echo "Getting the superbuild cache took " $(($SECONDS - $START_TIME)) "s"
