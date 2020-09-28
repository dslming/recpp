# 移除所有编译的内容
rm -rf build
# 创建文件夹
mkdir build
cd build
# 生成makefile
cmake ..
# 编译
make
cp ../../../data/Kinect2_RGB.pcd ./
# 运行文件
./cloud_viewer
