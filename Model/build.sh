# no-gpu -----

#cd darknet
#mkdir build
#cd build/
#cmake ..
#make
#cd -

# gpu -----
cd darknet-gpu
mkdir build
cd build/
cmake ..
make
cd ../..

# darklib[no-gpu]
cp darknet-gpu/build/libdarknet.so  yolov4-learn/lib/

sudo rm -rf darknet-gpu/build
