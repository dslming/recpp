#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <vector>
#include <ctime>                        /* time */

int main( int argc, char** argv ) {
	srand( time( NULL ) );          /* 随机数 */
	time_t begin, end;
	begin = clock();                /* 开始计时 */


	/**
	 * 产生假的点云数据
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud_ptr( new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> &	cloud = *cloud_ptr;
	cloud.width	= 4000;       /* 数据点 */
	cloud.height	= 1;
	cloud.points.resize( cloud.width * cloud.height );
	for ( size_t i = 0; i < cloud.points.size(); ++i ){
		cloud.points[i].x	= 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y	= 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z	= 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	/*
	 * 3D点云显示
	 */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer );  
	viewer->setBackgroundColor(0, 0, 0);                                                                  
	viewer->addPointCloud(cloud_ptr, "base");                                                             
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "base");

	/**
	/* keTree 
	 */
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_ptr);
	// 随机定义一个 需要搜寻的点
	pcl::PointXYZ searchPoint;
	searchPoint.x	= 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y	= 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z	= 1024.0f * rand() / (RAND_MAX + 1.0f);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ret_color( cloud_out, 0, 0, 255 );  
	viewer->addPointCloud(cloud_out, ret_color, "ret");                                                             
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "ret");

	// K 个最近点去搜索
	int K = 10;
    // 最近临搜索得到的索引
	std::vector<int>	pointIdxNKNSearch( K );        
	// 平方距离
	std::vector<float>	pointNKNSquaredDistance( K );   
	std::cout	<< "K nearest neighbor search at (" << searchPoint.x
			    << " " << searchPoint.y
			    << " " << searchPoint.z
			     << ") with K=" << K << std::endl;
	// 参考点
	pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud_searchPoint( new pcl::PointCloud<pcl::PointXYZ>);
	cloud_ref->push_back(searchPoint)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ret_color( cloud_out, 0, 0, 255 );  
	viewer->addPointCloud(cloud_out, ret_color, "searchPoint");                                                             
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "searchPoint");

	if ( kdtree.nearestKSearch( searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance ) > 0 ){
		for ( size_t i = 0; i < pointIdxNKNSearch.size(); ++i ) {
			pcl::PointXYZ point;
			point.x = cloud.points[pointIdxNKNSearch[i]].x;
			point.y = cloud.points[pointIdxNKNSearch[i]].y;
			point.z = cloud.points[pointIdxNKNSearch[i]].z;
			cloud_out->push_back( point );
		    // pointNKNSquaredDistance[i]
		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ret_color( cloud_out, 0, 255, 0 );  
		viewer->addPointCloud(cloud_out, ret_color, "ret");                                                             
    	viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "ret");
	}

	/* -------------------------------------------------------------------------------------------- */
	end = clock();                                          /* 结束计时 */
	double Times = double(end - begin) / CLOCKS_PER_SEC;    /* 将clock()函数的结果转化为以秒为单位的量 */
	std::cout << "time: " << Times << "s" << std::endl;

	while ( !viewer->wasStopped() )
	{
		viewer->spinOnce();
		pcl_sleep( 0.1 );
	}
	return(0);
}
