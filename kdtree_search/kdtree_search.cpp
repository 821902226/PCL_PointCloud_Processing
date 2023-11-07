#include <iostream>
#include <cstdlib>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

int main(void)
{
	//��ȡ�����ļ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("../bunny.pcd", *cloud);
	//�漴����һ����
	srand(time(0));
	pcl::PointXYZ search_point;
	search_point.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	search_point.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	search_point.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	
	//����KdTreeFLANN�������������ռ�
	pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
	kd_tree.setInputCloud(cloud);

	int k = 10;	//����10����
	std::vector<int> point_search(k);	//�洢���ڵ������
	std::vector<float> point_distance(k);	//�洢���ڵ��Ӧ��ƽ������
	float radius = 256.0f * rand() / (1.0f + RAND_MAX);	//���������뾶

	//ִ��K��������
	if (kd_tree.nearestKSearch(search_point, k, point_search, point_distance) > 0)	//���ҵ�����0������
	{
		std::cout << "K�������������" << std::endl;
		for (int i = 0; i < point_search.size(); ++i)
		{
			std::cout << cloud->points[point_search[i]].x << " " << cloud->points[point_search[i]].y
				<< " " << cloud->points[point_search[i]].z << " distance: " << point_distance[i]
				<< std::endl;
		}
	}
	else
	{
		std::cout << "δ��������ؽ��ڵ�" << std::endl;
	}

	std::cout << "*************************************************" << std::endl;

	//ִ�а뾶radius�ڽ�������
	if (kd_tree.radiusSearch(search_point, radius, point_search, point_distance) > 0)	//���ҵ�����0������
	{
		std::cout << "K�������������" << std::endl;
		for (int i = 0; i < point_search.size(); ++i)
		{
			std::cout << cloud->points[point_search[i]].x << " " << cloud->points[point_search[i]].y
				<< " " << cloud->points[point_search[i]].z << " distance: " << point_distance[i]
				<< std::endl;
		}
	}
	else
	{
		std::cout << "�ڰ뾶radius��δ��������ؽ��ڵ�" << std::endl;
	}
}