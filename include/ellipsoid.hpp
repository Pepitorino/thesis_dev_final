#pragma once 
#include "structs.hpp"
#include <Eigen/Dense>
#include <CGAL/Cartesian_d.h>
#include <CGAL/MP_Float.h>
#include <CGAL/point_generators_d.h>
#include <CGAL/Approximate_min_ellipsoid_d.h>
#include <CGAL/Approximate_min_ellipsoid_d_traits_d.h>
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ellipsoid
{
public:
    std::vector<std::vector<Eigen::Vector3d>> gmm_clustering();
    std::vector<EllipsoidParam> ellipsoidize_clusters_CGAL(
        const std::vector<std::vector<Eigen::Vector3d>> frontier_clusters,
        const std::vector<std::vector<Eigen::Vector3d>> clusters
    );
};