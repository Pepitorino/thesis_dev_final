#pragma once
#include "structs.hpp"
#include <string>
#include <Eigen>
#include <open3d/Open3D.h>
#include <octomap/Pointcloud.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

class voxelstruct
{
public:
    voxelstruct();
    //each point cloud will be cropped on its way in nalang
    void insertPointCloud(open3d::geometry::PointCloud pcd_new);
    void cropbbx(Eigen::Vector3d bbx_max, Eigen::Vector3d bbx_min);
    void classifyvoxels();
private:
    double resolution;
    
    octomap::ColorOctree* tree;
    open3d::geometry::PointCloud* current_pcd;
    std::vector<open3d::geometry::PointCloud*> pcd_list;

    std::vector<Eigen::Vector3d> surface_frontiers;
    std::vector<Eigen::Vector3d> occupied_voxels;
    std::vector<Eigen::Vector3d> roi_surface_frontier;
}