#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <vector>
#include <ctime>                        /* time */
using namespace std;
void showPoint(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
	int color[3],
	int pointSize,
	string cloudName 
	) {
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color( cloud, color[0], color[1],color[2]);  
    viewer->addPointCloud(cloud, cloud_color, cloudName);                                                             
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudName);
}


int main( int argc, char** argv ) {
	srand( time( NULL ) );          /* 随机数 */
	time_t begin, end;
	begin = clock();                /* 开始计时 */
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer );
	viewer->setBackgroundColor(0, 0, 0); 

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
	int base_color[3] = {255,255,255};                                                                 
	showPoint(cloud_ptr, viewer, base_color, 1, "base");

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

	// 显示搜寻的参考点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_searchPoint(new pcl::PointCloud<pcl::PointXYZ>());
	cloud_searchPoint->push_back(searchPoint);
	int search_color[3] = {255,0,0};
	showPoint(cloud_searchPoint, viewer, search_color, 6, "searchPoint");

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
	// 显示搜索结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
	if ( kdtree.nearestKSearch( searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance ) > 0 ){
		for ( size_t i = 0; i < pointIdxNKNSearch.size(); ++i ) {
			pcl::PointXYZ point;
			point.x = cloud.points[pointIdxNKNSearch[i]].x;
			point.y = cloud.points[pointIdxNKNSearch[i]].y;
			point.z = cloud.points[pointIdxNKNSearch[i]].z;
			cloud_out->push_back( point );
		    // pointNKNSquaredDistance[i]
		}
		int ret_color[3] = {0,255,0};
		showPoint(cloud_out, viewer, ret_color, 4, "ret");
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
