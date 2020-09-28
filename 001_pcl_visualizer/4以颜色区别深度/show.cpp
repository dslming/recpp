#include <iostream>    //所需头文件
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>    //点云查看窗口头文件
#include <pcl/io/io.h>

using namespace std;
using namespace pcl;
using namespace io;

int main() {
	// RGB点云
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    // 读取点云数据
	if (io::loadPCDFile("rabbit.pcd", *cloud) == -1){
		cerr << "can't read file file.pcd" << endl;
		return -1;
	}

	// 创建查看器对象
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

	// 设置背景颜色
	viewer->setBackgroundColor (0, 0, 0);

	// 创建一个颜色处理程序对象, 按照z字段进行渲染
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");

    // 添加点云
	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud1");

    // 添加坐标轴
	viewer->addCoordinateSystem (1.0);
	
	while (!viewer->wasStopped())  {
		viewer->spinOnce(100);
	}

	return 0;
}
