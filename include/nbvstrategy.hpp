#pragma once
#include "voxelstruct.hpp"
#include "ellipsoid.hpp"
#include "structs.hpp"
#include <Eigen/Dense>
#include <vector>

class nbvstrategy 
{
public:
    nbvstrategy();
    int initialize();
    Eigen::Matrix4d getCameraPose(const Eigen::Matrix<double,5,1> &vp);
    std::vector<Eigen::Vector3d> projectEllipsoidstoImage(
        const std::vector<EllipsoidParam> &ellipsoids,
        const Eigen::Matrix4d,
        const Camera &cam);
private:
    voxelstruct* voxel_struct;
    ellipsoid* ellipsoid_fitting;

    Camera cam;
    Eigen::Vector3d bbx_min, bbx_max;
    std::vector<PlantBBX> bbx_plants;
    int min_clusters, max_clusters;
    double resolution;
    double dx, dy, dz, dyaw, dpitch;

    void generateViewpoints();
    std::vector<Eigen::Matrix<double,5,1>> viewpoints;
};