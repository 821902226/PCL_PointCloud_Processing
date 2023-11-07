#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(void)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../bunny.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file bunny.pcd!!!");
		return(0);
	}
	pcl::io::savePCDFileASCII("../../new.pcd", *cloud);

	return(0);
}