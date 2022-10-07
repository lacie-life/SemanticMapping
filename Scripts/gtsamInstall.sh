#!/bin/bash

echo "GTSAM Install"

cd .

sudo apt-get -y install libboost-all-dev

sudo apt-get -y install libtbb-dev

wget https://github.com/borglab/gtsam/archive/refs/tags/4.0.3.tar.gz

tar zxf 4.0.3.tar.gz

cd gtsam-4.0.3

mkdir build

cd build

cmake ..

make -j8

sudo make install

cd ../..

sudo rm -rf gtsam-4.0.3

sudo rm 4.0.3.tar.gz

echo "GTSAM Install Success"

