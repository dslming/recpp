rm -rf build
mkdir build
cd build
cmake ..
make
mv ../*.pcd ./
./cloud_viewer
