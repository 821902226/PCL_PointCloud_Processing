#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
using namespace std;
int main(int argc, char* argv[])
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::io::loadPLYFile("../input/1.ply", *Cloud);//读入点云数据
 
    pcl::visualization::PCLVisualizer viewer("display");
    viewer.setBackgroundColor(255, 255, 255);
    
    viewer.addPointCloud<pcl::PointXYZRGB>(Cloud, "sample");//显示点云

    int num_points = Cloud->size();
    for (int i = 0; i < num_points; i++)
    {
        float x = Cloud->points[i].x;
        float y = Cloud->points[i].y;
        float z = Cloud->points[i].z;

        //添加球体到可视化器，以表示点
        viewer.addSphere(pcl::PointXYZ(x, y, z), 0.15, 0.2, 0.6, 1, "sphere_" + std::to_string(i));
    }

    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");//设置点云大小
 
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
 
 
    return 0;
}