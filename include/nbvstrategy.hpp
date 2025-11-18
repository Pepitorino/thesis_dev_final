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
    void destroy();
    int initialize(std::string settings_path);
    void getNBV(std::string view_file_path, 
    double x, double y, double z, 
    double yaw,
    double pitch);

private:
    Eigen::Matrix4d getCameraPose(const Eigen::Matrix<double,5,1> &vp);

    open3d::geometry::PointCloud T_cam_pcd_to_world(
        const Eigen::Matrix4d &T_cam_world, 
        const open3d::geometry::PointCloud* pcd);

    std::pair<double, cv::Mat> projectEllipsoidstoImage(
        const std::vector<EllipsoidParam> &ellipsoids,
        const Eigen::Matrix4d &T_cam_world);
    
    Eigen::Matrix4d create_ellipsoid_dual_matrix(
        const EllipsoidParam &param);
    
    Eigen::Matrix3d compute_ellipsoid_projection(
        const Eigen::Matrix<double, 3, 4> camera_matrix,
        const Eigen::Matrix4d ellipsoid_matrix_dual);
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

    std::vector<EllipsoidParam> ellipsoid_parameters;
};