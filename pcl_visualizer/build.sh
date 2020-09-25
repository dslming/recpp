rm -rf build
mkdir build
cd build
cmake ..
make
mv ../rabbit.pcd ./
./cloud_viewer
