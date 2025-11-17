#pragma once
#include "structs.hpp"
#include <string>
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <octomap/Pointcloud.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

class voxelstruct
{
public:
    voxelstruct(double resolution);
    //each point cloud will be cropped on its way in nalang
    void cropBBX(const Eigen::Vector3d& bbx_max, const Eigen::Vector3d& bbx_min, open3d::geometry::PointCloud* cloud);
    void insertPointCloud(open3d::geometry::PointCloud* pcd, Eigen::Vector3d camera);
    void classifyVoxels();
    std::vector<Eigen::Vector3d> getSurfaceFrontiers();
    std::vector<Eigen::Vector3d> getOccupiedVoxels();
    std::vector<Eigen::Vector3d> getROISurfaceFrontiers();
    int size();

private:
    double resolution;
    
    octomap::ColorOcTree* tree;
    open3d::geometry::PointCloud* pcd;
    std::vector<open3d::geometry::PointCloud*> pcd_list;

    std::vector<Eigen::Vector3d> surface_frontiers;
    std::vector<Eigen::Vector3d> occupied_voxels;
    std::vector<Eigen::Vector3d> roi_surface_frontier;
};