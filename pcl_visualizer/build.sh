rm -rf build
mkdir build
cd build
cmake ..
make
cp ../*.pcd ./
./cloud_viewer
