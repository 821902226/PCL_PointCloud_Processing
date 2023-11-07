#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>


int
main(int argc, char** argv)
{
    //���ص�һ������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../B1.pcd", *target_cloud);

    //���صڶ�������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../C2.pcd", *input_cloud);

    //�������˺�ĵ��ƶ���
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //��ʼ����̬�ֲ��任��NDT��
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;
    //���������߶�NDT����
    //Ϊ��ֹ����������Сת������
    ndt.setTransformationEpsilon(0.01);
    //ΪMore-Thuente������������󲽳�
    ndt.setStepSize(0.1);
    //����NDT����ṹ�ķֱ��ʣ�VoxelGridCovariance��
    ndt.setResolution(1.0);
    //����ƥ�������������
    //ndt.setMaximumIterations(500);
    // ����Ҫ��׼�ĵ���
    ndt.setInputSource(input_cloud);
    //���õ�����׼Ŀ��
    ndt.setInputTarget(target_cloud);
    //������ת�����ƽ�ƾ��������׼
    ndt.align(*output_cloud);
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;

    //����ת�����������
    pcl::io::savePCDFileASCII("../box_transformed.pcd", *output_cloud);

    //---------------------------���ƿ��ӻ�-------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.5, 0.5, 0.5);
    //��Ŀ����ƿ��ӻ�
    viewer->addPointCloud<pcl::PointXYZRGB>(target_cloud,"target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    //��ת�����Ŀ����ƿ��ӻ�
    viewer->addPointCloud<pcl::PointXYZRGB>(output_cloud, "output cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");
    // �������ӻ�
    viewer->initCameraParameters();
    //�ȴ�ֱ�����ӻ����ڹر�
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    return (0);
}