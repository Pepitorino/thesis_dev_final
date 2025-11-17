#include "voxelstruct.hpp"
#include <open3d/Open3D.h>
#include <octomap/Pointcloud.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <vector>
#include <string>

voxelstruct::voxelstruct(double resolution) 
{
    this->resolution = resolution;
    this->tree = new octomap::ColorOcTree(resolution);
}

// for further optimization
void voxelstruct::cropBBX(
    const Eigen::Vector3d& bbx_min,
    const Eigen::Vector3d& bbx_max,
    open3d::geometry::PointCloud* cloud)
{
    if (!cloud || cloud->points_.empty())
        return;

    auto& points = cloud->points_;
    auto& colors = cloud->colors_;

    std::vector<Eigen::Vector3d> new_points;
    std::vector<Eigen::Vector3d> new_colors;

    new_points.reserve(points.size());
    if (!colors.empty())
        new_colors.reserve(colors.size());

    for (size_t i = 0; i < points.size(); ++i) {
        const Eigen::Vector3d& p = points[i];

        if ((p.array() >= bbx_min.array()).all() &&
            (p.array() <= bbx_max.array()).all()) {

            new_points.push_back(p);
            if (!colors.empty()) {
                new_colors.push_back(colors[i]);
            }
        }
    }

    points.swap(new_points);
    if (!colors.empty())
        colors.swap(new_colors);
}

// for further optimization
void voxelstruct::insertPointCloud(
    open3d::geometry::PointCloud* pcd,
    Eigen::Vector3d camera) 
{
    if(!pcd||pcd->points_.empty()) return;
    
    this->pcd = pcd;
    this->pcd_list.push_back(pcd);

    for (size_t i = 0; i < pcd->points_.size(); ++i) {
        const auto& p = pcd->points_[i];
        const auto& c = pcd->colors_[i];

        octomap::point3d origin(camera(0), camera(1), camera(2));
        octomap::point3d endpoint(p(0), p(1), p(2));

        // Insert ray (marks free voxels + marks endpoint occupied)
        tree->insertRay(origin, endpoint);

        // Now add color to the occupied endpoint
        unsigned char r = static_cast<unsigned char>(c(0) * 255);
        unsigned char g = static_cast<unsigned char>(c(1) * 255);
        unsigned char b = static_cast<unsigned char>(c(2) * 255);

        tree->integrateNodeColor(endpoint.x(), endpoint.y(), endpoint.z(), r, g, b);
    }

    tree->updateInnerOccupancy();
}

// for further optimization
void voxelstruct::classifyVoxels() {
    this->surface_frontiers.clear();
    this->occupied_voxels.clear();
    this->roi_surface_frontier.clear();

    const double res = this->resolution;
    const int dirs[6][3] = {
        {1, 0, 0}, {-1, 0, 0},
        {0, 1, 0}, {0, -1, 0},
        {0, 0, 1}, {0, 0, -1}
    };

    for (auto it = this->tree->begin_leafs(), end = this->tree->end_leafs(); it != end; ++it) {
        const octomap::ColorOcTreeNode* node = &(*it);

        // Only consider occupied voxels
        if (!tree->isNodeOccupied(*it)) continue;

        Eigen::Vector3d center(it.getX(), it.getY(), it.getZ());

        // Check if the voxel is red
        octomap::ColorOcTreeNode::Color c = node->getColor();
        bool is_red = (c.r >= 140);

        bool is_surface_frontier = false;

        for (auto &d : dirs) {
            octomap::point3d neighbor(it.getX() + d[0]*res,
                                    it.getY() + d[1]*res,
                                    it.getZ() + d[2]*res);

            octomap::OcTreeKey key;
            if (tree->coordToKeyChecked(neighbor, key)) {
                auto* neighbor_node = tree->search(key);

                // Check neighbor: it must exist
                if (neighbor_node == nullptr) {
                    // Unknown neighbor → candidate for surface frontier
                    is_surface_frontier = true;
                } else if (!tree->isNodeOccupied(*neighbor_node)) {
                    // Neighbor is free → NOT a frontier
                    is_surface_frontier = false;
                    break;
                }
                // Occupied neighbor → OK
            }
        }

        // Classify
        if (is_red && is_surface_frontier) {
            this->roi_surface_frontier.push_back(center);
        } else if (!is_red && is_surface_frontier) {
            this->surface_frontiers.push_back(center);
        } else {
            this->occupied_voxels.push_back(center);
        }
    }
}

std::vector<Eigen::Vector3d> voxelstruct::getSurfaceFrontiers() {
    return this->surface_frontiers;
};  

std::vector<Eigen::Vector3d> voxelstruct::getOccupiedVoxels() {
    return this->occupied_voxels;
};

std::vector<Eigen::Vector3d> voxelstruct::getROISurfaceFrontiers() {
    return this->roi_surface_frontier;
};

int voxelstruct::size() {
    return this->tree->size();
}