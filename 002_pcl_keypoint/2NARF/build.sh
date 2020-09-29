# 移除所有编译的内容
rm -rf build
# 创建文件夹
mkdir build
cd build
# 生成makefile
cmake ..
# 编译
make
# 将 pcd 文件拷贝到build, pcd文件要与执行文件相同的路径
cp ../../../data/frame_00000.pcd ./
# 运行文件
./cloud_viewer
