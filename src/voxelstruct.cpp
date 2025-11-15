#pragma once
#include <open3d/Open3d.h>
#include <octomap/Pointcloud.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <vector>
#include <string>

voxelstruct::voxelstruct(double resolution) 
{
    this->resolution = resolution;
    this->tree = new octomap::ColorOctree(resolution);
}

void voxelstruct::insertPointCloud(
    open3d::geometry::PointCloud pcd
    Eigen::Vector3d camera) 
{
    for (size_t i = 0; i < pcd.points_.size(); ++i) {
        const auto& p = pcd.points_[i];
        const auto& c = pcd.colors_[i];

        octomap::point3d origin(camera[0], camera[1], camera[2]);
        octomap::point3d endpoint(p(0), p(1), p(2));

        // Insert ray (marks free voxels + marks endpoint occupied)
        tree.insertRay(origin, endpoint);

        // Now add color to the occupied endpoint
        unsigned char r = static_cast<unsigned char>(c(0) * 255);
        unsigned char g = static_cast<unsigned char>(c(1) * 255);
        unsigned char b = static_cast<unsigned char>(c(2) * 255);

        tree.integrateNodeColor(endpoint.x(), endpoint.y(), endpoint.z(), r, g, b);
    }

    tree.updateInnerOccupancy();
}