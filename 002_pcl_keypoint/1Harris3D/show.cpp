#include <iostream>            
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>     
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/harris_3d.h>
#include <cstdlib>
#include <vector>
using namespace std;

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
    pcl::io::loadPCDFile ("rabbit.pcd", point_cloud);      
    cout << "load pcd file : " << "rabbit.pcd" << endl;
    cout << "point_cloud has :" << point_cloud.points.size() << " n points." << endl;


	/*
	 * 3D点云显示
	 */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer );   /* 可视化对象指针 */
	viewer->setBackgroundColor(0, 0, 0);                                                                  /* 背景颜色　白色 */
	viewer->addPointCloud( point_cloud_ptr );                                                               /* 指针 */

	/*
	 * --------------------------------
	 * -----Extract Harri keypoints-----
	 * --------------------------------
	 * 提取Harri关键点
	 */
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	harris.setInputCloud( point_cloud_ptr );                                                                /* 设置输入点云 指针 */
	cout << "input successful" << endl;
	harris.setNonMaxSupression( true );
	harris.setRadius( 0.02f );                                                                              /* 　块体半径 */
	harris.setThreshold( 0.01f );                                                                           /* 数量阈值 */
	cout << "parameter set successful" << endl;

	/*
	 * 新建的点云必须初始化，清零，否则指针会越界
	 * 注意Harris的输出点云必须是有强度(I)信息的 pcl::PointXYZI，因为评估值保存在I分量里
	 */
	pcl::PointCloud<pcl::PointXYZI>::Ptr	cloud_out_ptr( new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI> &	cloud_out = *cloud_out_ptr;
	cloud_out.height	= 1;
	cloud_out.width		= 100;
	cloud_out.resize( cloud_out.height * cloud_out.width );
	cloud_out.clear();
	cout << "extracting... " << endl;

	/* 计算特征点 */
	harris.compute( cloud_out );
	/* 关键点 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud_harris_ptr( new pcl::PointCloud<pcl::PointXYZ>);  /* 指针 */
	pcl::PointCloud<pcl::PointXYZ> &	cloud_harris = *cloud_harris_ptr;                       /* 引用 */
	cloud_harris.height	= 1;
	cloud_harris.width	= 100;
	cloud_harris.resize( cloud_out.height * cloud_out.width );
	cloud_harris.clear();                                                                           /* 清空 */
	int size = cloud_out.size();
	cout << "extraction : " << size << " n keypoints." << endl;
	pcl::PointXYZ point;
	/* 可视化结果不支持XYZI格式点云，所有又要导回XYZ格式。。。。 */
	for ( int i = 0; i < size; ++i )
	{
		point.x = cloud_out.at( i ).x;
		point.y = cloud_out.at( i ).y;
		point.z = cloud_out.at( i ).z;
		cloud_harris.push_back( point );
	}
	/*
	 * -------------------------------------
	 * -----Show keypoints in 3D viewer-----
	 * -------------------------------------
	 * 在3D图形窗口中显示关键点
	 */
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler( cloud_harris_ptr, 0, 255, 0 );    /* 第一个参数类型为　指针 */
	viewer->addPointCloud( cloud_harris_ptr, harris_color_handler, "harris" );                                              /* 第一个参数类型为　指针 */
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
