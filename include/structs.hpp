#pragma once
#include <Eigen/Dense>
#include <string>

struct EllipsoidParam {
    Eigen::Matrix4d pose;   // 4x4 transformation matrix (rotation + translation)
    Eigen::Vector3d radii;  // Semi-axis lengths of the ellipsoid
    std::string type;       // "frontier" or "occupied"
};

struct Camera {
    int width = 1920;
    int height = 1080;
    double fx = 1000.0;  // focal length in pixels
    double fy = 1000.0;
    double cx = width / 2.0;
    double cy = height / 2.0;
    double max_range = 2.0; // meters
};

struct PlantBBX {
    Eigen::Vector3d min;
    Eigen::Vector3d max;
}