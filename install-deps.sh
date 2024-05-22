#!/bin/sh

## Modify the following lines to install dependencies for your project.
echo "Installing dependencies for non-ROS packages"
apt update -q
apt install -y --no-install-recommends git libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev

mkdir ./deps
git clone --depth 1 --branch 2.2.0 https://ceres-solver.googlesource.com/ceres-solver ./deps/ceres-solver
cd ./deps/ceres-solver
mkdir build && cd build
cmake .. && make -j10 && make install
cd ../../..
rm -rf ./deps