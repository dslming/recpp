#include <iostream>    //所需头文件
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>    //点云查看窗口头文件
#include <pcl/io/io.h>

using namespace std;
using namespace pcl;
using namespace io;

int main() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source1(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source2(new pcl::PointCloud<pcl::PointXYZRGB>());

	//输入点云路径
	string filename1 = "rabbit.pcd";
	string filename2 = "rabbit.pcd";

	// 读取文件
	int ret1 = pcl::io::loadPCDFile(filename1, *source1);
	int ret2 = pcl::io::loadPCDFile(filename2, *source2);
	if (ret1 == -1 || ret2 == -1){
		cerr << "点云加载失败..." << endl;
		return -1;
	} else {
		cout << "点云加载成功..." << endl;
	}

	// 创建查看器对象
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(source1,255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(source1, rgb, "sample cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(source2, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(source2, single_color, "sample cloud2", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");

	viewer->addCoordinateSystem(1.0); 
	viewer->spin();

	while (!viewer->wasStopped())  {
		viewer->spinOnce(100);
	}

	return 0;
}
