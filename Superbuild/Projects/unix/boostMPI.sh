#!/bin/bash
# run by Superbuild Makefile.txt
echo "Add MPI support"

cd boost/src/boost/

# update configuration file
echo "using mpi ;" >> project-config.jam 

# compile library
./b2 -j$(nproc) --target=shared,static

# install library
./b2 install

cd ../../..

echo "MPI support completed"
