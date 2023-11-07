#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>


int
main(int argc, char** argv)
{
    //加载第一个点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../B1.pcd", *target_cloud);

    //加载第二个点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../C2.pcd", *input_cloud);

    //创建过滤后的点云对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //初始化正态分布变换（NDT）
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt.setTransformationEpsilon(0.01);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize(0.1);
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setResolution(1.0);
    //设置匹配迭代的最大次数
    //ndt.setMaximumIterations(500);
    // 设置要配准的点云
    ndt.setInputSource(input_cloud);
    //设置点云配准目标
    ndt.setInputTarget(target_cloud);
    //计算旋转矩阵和平移矩阵进行配准
    ndt.align(*output_cloud);
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;

    //保存转换的输入点云
    pcl::io::savePCDFileASCII("../box_transformed.pcd", *output_cloud);

    //---------------------------点云可视化-------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.5, 0.5, 0.5);
    //对目标点云可视化
    viewer->addPointCloud<pcl::PointXYZRGB>(target_cloud,"target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    //对转换后的目标点云可视化
    viewer->addPointCloud<pcl::PointXYZRGB>(output_cloud, "output cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");
    // 启动可视化
    viewer->initCameraParameters();
    //等待直到可视化窗口关闭
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    return (0);
}