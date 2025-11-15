#pragma once
#include "voxelstruct.hpp"
#include "ellipsoid.hpp"
#include "structs.hpp"
#include <Eigen/Dense>
#include <vector>

class nbvstrategy 
{
public:
    nbvstrategy(double resolution);
    Eigen::Matrix4d getCameraPose(const Eigen::Matrix<double,5,1> &vp);
    std::vector<Eigen::Vector3d> projectEllipsoidstoImage(
        const std::vector<EllipsoidParam> &ellipsoids,
        const Eigen::Matrix4d,
        const Camera &cam);
private:
    voxelstruct* voxel_struct;
    ellipsoid ellipsoid_fitting;
    Camera cam_parameters;

    void generateViewpoints();
    std::vector<Eigen::Matrix<double,5,1>> viewpoints;

    Eigen::Vector<double,5> xyzypgenf;
    Eigen::Vector3d bbx_min, bbx_max;
    std::vector<Eigen::Vector3d> bbx_plants_min, bbx_plants_max;
};