// 该方法(PointCloudColorHandlerRGBField)要求点云类型包含RGB三个颜色分量，即该方法是直接显示点云中各个点的RGB属性字段信息，而不是通过对点云着色显示不同颜色。

#include <iostream>    //所需头文件
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>    //点云查看窗口头文件
#include <pcl/io/io.h>

using namespace std;
using namespace pcl;
using namespace io;

int main() {
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);   //创建点云指针

	if (io::loadPLYFile("SphereDivision.ply", *cloud) == -1){ 
		cerr << "can't read file file.pcd" << endl;
		return -1;
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud); 
    
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb, "sample cloud");  
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

	while (!viewer->wasStopped())  {
		// viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
