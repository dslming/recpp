rm -rf build
mkdir build
cd build
cmake ..
make
# 将可执行文件移出来
mv ./cloud_viewer ../
./ cloud_viewer