#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int main(void)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PLYReader reader;
	if (reader.read("../horse_color.ply", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read the pcd file\n");
		return(0);
	}
	pcl::PCDWriter writer;
	writer.write("../horse.pcd", *cloud);

	return(0);
}