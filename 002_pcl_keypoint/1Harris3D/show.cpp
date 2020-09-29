#include <iostream>            
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>     
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/harris_3d.h>
#include <cstdlib>
#include <vector>
using namespace std;
// https://blog.csdn.net/zzh_AI/article/details/92973574

/* 定义别名 */
typedef pcl::PointXYZ PointType;

/*
 * --------------
 * -----Main-----
 * --------------
 */
int main( int argc, char** argv ){
	/*
	 * 读取pcd文件
	 */
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr( new pcl::PointCloud<PointType>);       
	pcl::PointCloud<PointType> & point_cloud = *point_cloud_ptr;   
    pcl::io::loadPCDFile ("roorm.pcd", point_cloud);      
    cout << "load pcd file : " << "roorm.pcd" << endl;
    cout << "point_cloud has :" << point_cloud.points.size() << " n points." << endl;


	/*
	 * 3D点云显示
	 */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer );  
	viewer->setBackgroundColor(0, 0, 0);                                                                  
	viewer->addPointCloud(point_cloud_ptr, "base");                                                             
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "base");
	/*
	 * 提取Harri关键点
	 */
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	// harris.setNonMaxSupression( true );
	harris.setRadius( 0.9);                                                                             
	harris.setRadiusSearch(0.1);
	// harris.setThreshold( 0.1 );   
	harris.setInputCloud( point_cloud_ptr );                                                               
	cout << "parameter set successful" << endl;

	// 创建新点云,保存关键点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
    cloud_out->clear();
	harris.compute(*cloud_out);
    int key_size = cloud_out->size();
	cout << "extraction : " << key_size << "keypoints." << endl;
	cout << "cloud_out width:" << cloud_out->width << "." << endl;
	cout << "cloud_out height:" << cloud_out->height << "." << endl;

	/*
	 * 在3D图形窗口中显示关键点
	 */
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> harris_color_handler( cloud_out, 0, 255, 0 );   
	viewer->addPointCloud( cloud_out, harris_color_handler, "harris" );                                             
	viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris" );

	/*
	 * --------------------
	 * -----Main loop-----
	 * --------------------
	 */
	while ( !viewer->wasStopped() )
	{
		viewer->spinOnce();
		pcl_sleep( 0.1 );
	}
    return 0;
}
