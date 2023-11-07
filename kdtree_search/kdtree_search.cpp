#include <iostream>
#include <cstdlib>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

int main(void)
{
	//读取点云文件
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("../bunny.pcd", *cloud);
	//随即创建一个点
	srand(time(0));
	pcl::PointXYZ search_point;
	search_point.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	search_point.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	search_point.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	
	//创建KdTreeFLANN对象，设置搜索空间
	pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
	kd_tree.setInputCloud(cloud);

	int k = 10;	//查找10个点
	std::vector<int> point_search(k);	//存储近邻点的索引
	std::vector<float> point_distance(k);	//存储近邻点对应的平方距离
	float radius = 256.0f * rand() / (1.0f + RAND_MAX);	//设置搜索半径

	//执行K近邻搜索
	if (kd_tree.nearestKSearch(search_point, k, point_search, point_distance) > 0)	//查找到多余0个近邻
	{
		std::cout << "K近邻搜索结果：" << std::endl;
		for (int i = 0; i < point_search.size(); ++i)
		{
			std::cout << cloud->points[point_search[i]].x << " " << cloud->points[point_search[i]].y
				<< " " << cloud->points[point_search[i]].z << " distance: " << point_distance[i]
				<< std::endl;
		}
	}
	else
	{
		std::cout << "未搜索到相关近邻点" << std::endl;
	}

	std::cout << "*************************************************" << std::endl;

	//执行半径radius内近邻搜索
	if (kd_tree.radiusSearch(search_point, radius, point_search, point_distance) > 0)	//查找到多余0个近邻
	{
		std::cout << "K近邻搜索结果：" << std::endl;
		for (int i = 0; i < point_search.size(); ++i)
		{
			std::cout << cloud->points[point_search[i]].x << " " << cloud->points[point_search[i]].y
				<< " " << cloud->points[point_search[i]].z << " distance: " << point_distance[i]
				<< std::endl;
		}
	}
	else
	{
		std::cout << "在半径radius内未搜索到相关近邻点" << std::endl;
	}
}