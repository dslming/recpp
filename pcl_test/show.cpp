#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //
    //*打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file rabbit.pcd\n");
        return(-1);
    }
    std::cout << cloud->points.size() << std::endl;
    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {

    }
    system("pause");
    return 0;
}