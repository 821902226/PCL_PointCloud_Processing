#include <stdio.h>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/random.hpp>
using namespace std;
typedef pcl::PointXYZRGBNormal PointT;

void AddGuassNoise(pcl::PointCloud<PointT>::Ptr clean_cloud, float mu, float sigma)
{
	boost::mt19937 rng;	//等分布均匀伪随机发生器
	rng.seed(static_cast<unsigned int>(time(0)));	//随机种子
	boost::normal_distribution<> nd(mu, sigma);	//定义正态分布，均值为mu，标准差为sigma
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> guass_noise(rng, nd);	//生成高斯噪声

	pcl::PointCloud<PointT> noise_cloud;
	pcl::copyPointCloud(*clean_cloud, noise_cloud);
	for (size_t i = 0; i < clean_cloud->points.size(); i++)
	{
		noise_cloud.points[i].x += static_cast<float>(guass_noise());
		noise_cloud.points[i].y += static_cast<float>(guass_noise());
		noise_cloud.points[i].z += static_cast<float>(guass_noise());
	}

	*clean_cloud = *clean_cloud + noise_cloud;

}

int main()
{
	pcl::PointCloud<PointT>::Ptr clean_cloud(new pcl::PointCloud<PointT>);

	pcl::PLYReader reader;
	reader.read("../chicken.ply", *clean_cloud);

	float mu, sigma;	//均值、标准差
	mu = 0;
	sigma = 1.5;

	AddGuassNoise(clean_cloud, mu, sigma);

	pcl::io::savePLYFileASCII("../chicken_noise.ply", *clean_cloud);
	return 0;
}
