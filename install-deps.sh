#!/bin/sh

## Modify the following lines to install dependencies for your project.
echo "Installing dependencies for non-ROS packages"
sudo apt update -q
sudo apt install -y --no-install-recommends git libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev

mkdir ./deps
cd ./deps

git clone --depth 1 --branch 2.2.0 https://ceres-solver.googlesource.com/ceres-solver ./ceres-solver
cd ./ceres-solver
mkdir build && cd build
cmake .. && make -j10 && sudo make install
cd ../..

git clone --depth 1 --branch v1.x https://github.com/gabime/spdlog.git ./spdlog
cd ./spdlog && mkdir build && cd build
cmake .. && make -j10 && sudo make install
cd ../..

cd ../
rm -rf ./deps