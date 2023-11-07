#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int main(void)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	if (reader.read("../bunny.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read the pcd file\n");
		return(0);
	}
	pcl::PLYWriter writer;
	writer.write("../bunny.ply", *cloud);

	std::cout << "×ª»»³É¹¦£¡£¡£¡" << std::endl;
	return(0);
}