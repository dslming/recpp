/**
 * 点云可视化
 * @note pcd文件要和执行文件在相同文件夹
 * @note 路径不能出现中文
 */

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
    
int main () {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("rabbit.pcd", *cloud);
    
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    viewer.showCloud(cloud);
    while (!viewer.wasStopped ()) {}
    return 0;
}