#include "iostream"
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include "pcl/kdtree/kdtree_flann.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//计算均方根误差
float calculateRMSE(PointCloudT::Ptr source_cloud, PointCloudT::Ptr filtered_cloud)
{
    float rmse = 0.0f;

    pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>());
    tree->setInputCloud(source_cloud);

    for (int i = 0; i < filtered_cloud->size(); ++i) {
        std::vector<int> indices;
        std::vector<float> squared_distance;
        tree->nearestKSearch(filtered_cloud->at(i), 10, indices, squared_distance);
        for (int j = 0; j < 10; ++j) {
            float squared_dist = squared_distance[j];
            rmse += squared_dist;
        }
    }

    //rmse = std::sqrt(rmse / static_cast<float> (filtered_cloud->size()));
    rmse = std::sqrt(rmse / static_cast<float> (filtered_cloud->size() * 10));
    return rmse;
}

//计算平均距离
float calculateMD(PointCloudT::Ptr source_cloud, PointCloudT::Ptr filtered_cloud)
{
    float md = 0.0f;

    pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>());
    tree->setInputCloud(source_cloud);

    for (int i = 0; i < filtered_cloud->size(); ++i) {
        std::vector<int> indices;
        std::vector<float> squared_distance;
        tree->nearestKSearch(filtered_cloud->at(i), 1, indices, squared_distance);
        float squared_dist = sqrt(squared_distance[0]);
        md += squared_dist;
    }

    md = md / static_cast<float> (filtered_cloud->size());
    return md;
}

//计算倒角距离
float calculate_chamfer_distance(PointCloudT::Ptr source_cloud, PointCloudT::Ptr filtered_cloud)
{
    float chamfer_distance=0.f, chamfer_dist1=0.f, chamfer_dist2=0.f;
    std::vector<float> squared_dist1, squared_dist2;

    pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>());
    tree->setInputCloud(source_cloud);

    for (int i = 0; i < filtered_cloud->size(); ++i) {
        std::vector<int> indices;
        std::vector<float> squared_distance;
        tree->nearestKSearch(filtered_cloud->at(i), 1, indices, squared_distance);
        squared_dist1.push_back(squared_distance[0]);
    }

    tree->setInputCloud(filtered_cloud);
    for (int i = 0; i < source_cloud->size(); ++i) {
        std::vector<int> indices;
        std::vector<float> squared_distance;
        tree->nearestKSearch(source_cloud->at(i), 1, indices, squared_distance);
        squared_dist2.push_back(squared_distance[0]);
    }

    for (int i = 0; i < squared_dist1.size(); ++i) {
        chamfer_dist1 += squared_dist1[i];
    }

    for (int i = 0; i < squared_dist2.size(); ++i) {
        chamfer_dist2 += squared_dist2[i];
    }

    chamfer_distance = chamfer_dist1 / static_cast<float>(filtered_cloud->size()) + chamfer_dist2 / static_cast<float>(source_cloud->size());
    return chamfer_distance;
}

int main(int argc, char** argv)
{
    PointCloudT::Ptr source_cloud(new PointCloudT);
    PointCloudT::Ptr filtered_cloud(new PointCloudT);

    //读取源点云和过滤的点云
    //pcl::io::loadPCDFile("../bunny.pcd", *source_cloud);
    //pcl::io::loadPCDFile("../bunny_sor.pcd", *filtered_cloud);
    pcl::io::loadPLYFile("../groundtruth/armadillo.ply", *source_cloud);
    pcl::io::loadPLYFile("../result/armadillo_rimls.ply", *filtered_cloud);

    float rmse, md, chamfer_distance;

    //计算均方根误差
    rmse = calculateRMSE(source_cloud, filtered_cloud);

    //计算平均距离
    md = calculateMD(source_cloud, filtered_cloud);

    //计算倒角距离
    chamfer_distance = calculate_chamfer_distance(source_cloud, filtered_cloud);

    std::cout << "均方根误差： " << rmse << " 平均距离： " << md << "  倒角距离： " << chamfer_distance << std::endl;
}