#include <iostream>
#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include "nbvstrategy.hpp"
#include "json.hpp"

using json = nlohmann::json;

nbvstrategy::nbvstrategy() {

}

void nbvstrategy::destroy() {
    delete this->voxel_struct;
    delete this->ellipsoid_fitting;
    return; 
}

template<typename T>
T getOrDefault(const json& j, const std::string& key, T fallback) {
    if (j.contains(key) && !j[key].is_null()) {
        return j[key].get<T>();
    }
    return fallback;
}

std::vector<double> getVectorOrEmpty(const json& j, const std::string& key) 
{
    if (j.contains(key) && j[key].is_array())
        return j[key].get<std::vector<double>>();
    return {};
}

Eigen::Vector3d getVec3OrDefault(const json& j, const std::string& key) 
{
    if (j.contains(key)) {
        const auto& obj = j[key];
        if (obj.contains("x") && obj.contains("y") && obj.contains("z")) {
            return Eigen::Vector3d(obj["x"], obj["y"], obj["z"]);
        }
    }
    return Eigen::Vector3d::Zero();
}

int nbvstrategy::initialize(std::string settings_path) 
{
    std::ifstream file(settings_path);
    if(!file.is_open()) {
        std::cerr << "Failed to load settings";
        return -1;
    }

    json cfg;
    file >> cfg;

    //loads camera parameters
    auto cam = cfg["camera"];

    int width = getOrDefault<int>(cam, "width", 0);
    int height = getOrDefault<int>(cam, "height", 0);
    double fx = getOrDefault<double>(cam, "fx", 0);
    double fy = getOrDefault<double>(cam, "fy", 0);
    double cx = getOrDefault<double>(cam, "cx", 0);
    double cy = getOrDefault<double>(cam, "cy", 0);
    double min_range = getOrDefault<double>(cam, "min_range", 0);
    double max_range = getOrDefault<double>(cam, "max_range", 0);

    std::cout << "Camera loaded!" << std::endl;
    std::cout << "Parameters: \n" <<
        "Width: " << width << "\n" <<
        "Heigh: " << height << "\n" <<
        "fx: " << fx << "\n" <<
        "fy: " << fy << "\n" <<
        "cx: " << cx << "\n" <<
        "cy: " << cy << "\n" <<
        "Minimum Range: " << min_range << "\n" <<
        "Maximum Range: " << max_range << "\n" << std::endl;

    // loads global bbx
    auto bbx = cfg["bounding_box"];
    std::vector<double> bbx_min = getVectorOrEmpty(bbx, "bbx_min");
    std::vector<double> bbx_max = getVectorOrEmpty(bbx, "bbx_max");

    std::cout << "Bounding Box loaded!" << std::endl;
    std::cout << "Parameters: \n" <<
        "Minimum Bounding Box (x,y,z): " << bbx_min[0] << ", " << bbx_min[1] << ", " << bbx_min[2] << "\n" <<
        "Maximum Bounding Box (x,y,z): " << bbx_max[0] << ", " << bbx_max[1] << ", " << bbx_max[2] << std::endl << std::endl;        

    // loads plant bbxs
    std::vector<PlantBBX> plant_bbx_list;
    auto plants_region = cfg["plants_region"];
    for (auto& p : plants_region["plants"]) {
        PlantBBX pb;
        pb.min = getVec3OrDefault(p, "min");
        pb.max = getVec3OrDefault(p, "max");
        plant_bbx_list.push_back(pb);
    }
    
    std::cout << "Plants Bounding Boxes loaded! " << std::endl;
    for (auto& p : plant_bbx_list) {
        std::cout << "Plant Bounding Box (x,y,z): " << p.min[0] << ", " << p.min[1] << ", " << p.min[2] << std::endl;
        std::cout << "Plant Bounding Box (x,y,z): " << p.max[0] << ", " << p.max[1] << ", " << p.max[2] << std::endl << std::endl;
    }   

    // loads cluster numbers;
    auto clustering = cfg["clustering"];
    int min_clusters = getOrDefault(clustering, "min_clusters", 2);
    int max_clusters = getOrDefault(clustering, "max_clusters", 10);

    std::cout << "Cluster sized loaded! " << std::endl;
    std::cout << "Minimum Clusters: " << min_clusters << std::endl;
    std::cout << "Maximum Clusters: " << max_clusters << std::endl << std::endl;

    // loads octomap res
    auto octomap = cfg["octomap"];
    double resolution = getOrDefault(octomap, "resolution",  0.05);

    std::cout << "Octomap resolution loaded! " << std::endl;
    std::cout << "Resolution: " << resolution << std::endl << std::endl;

    // loads viewpoint generation frequency
    auto xyzypgenf = cfg["xyzypgenf"];
    double dx = getOrDefault(xyzypgenf, "dx", 0.05);
    double dy = getOrDefault(xyzypgenf, "dy", 0.05);
    double dz = getOrDefault(xyzypgenf, "dz", 0.05);
    double dyaw = getOrDefault(xyzypgenf, "dyaw", 4);
    double dpitch = getOrDefault(xyzypgenf, "dpitch", 4);

    std::cout << "Viewpoint Generation Frequency loaded!" << std::endl;
    std::cout << "dx: " << dx << std::endl;
    std::cout << "dy: " << dy << std::endl;
    std::cout << "dz: " << dz << std::endl;
    std::cout << "dyaw: " << dyaw << std::endl;
    std::cout << "dpitch: " << dpitch << std::endl << std::endl;

    // assigning to class members
    // camera
    this->cam.width = width;
    this->cam.height = height;
    this->cam.fx = fx;
    this->cam.fy = fy;
    this->cam.cx = cx;
    this->cam.cy = cy;
    this->cam.max_range = max_range;

    // global bbx
    if (bbx_min.size() != 3 || bbx_max.size() != 3) throw std::runtime_error("bbx_min or bbx_max must have 3 elements");
    Eigen::Vector3d bbx_min_eigen(bbx_min[0],bbx_min[1],bbx_min[2]);
    this->bbx_min = bbx_min_eigen;
    Eigen::Vector3d bbx_max_eigen(bbx_max[0],bbx_max[1],bbx_max[2]);
    this->bbx_max = bbx_max_eigen;

    // bounding box
    this->bbx_plants = plant_bbx_list;

    // gmm clusters
    this->min_clusters = min_clusters;
    this->max_clusters = max_clusters;

    // octomap resolution
    this->resolution = resolution;

    // viewpoint generation frequency
    this->dx = dx;
    this->dy = dy;
    this->dz = dz;
    this->dyaw = M_PI/dyaw;
    this->dpitch = M_PI/dpitch;

    this->voxel_struct = new voxelstruct(this->resolution);
    this->ellipsoid_fitting = new ellipsoid(this->min_clusters, this->max_clusters);

    this->generateViewpoints();

    std::cout << "Viewpoints Generated!" << std::endl;
    std::cout << "Number of viewpoints: " << this->viewpoints.size() << std::endl;
    std::cout << "First viewpoint: " << viewpoints.front().transpose() << std::endl;
    std::cout << "Last viewpoint: " << viewpoints.back(). transpose() << std::endl;

    return 0;
}

void nbvstrategy::generateViewpoints() 
{
    size_t nx = (this->bbx_max[0] - this->bbx_min[0])/this->dx+1;
    size_t ny = (this->bbx_max[0] - this->bbx_min[0])/this->dx+1;
    size_t nz = (this->bbx_max[0] - this->bbx_min[0])/this->dx+1;
    size_t nyaw = (this->bbx_max[0] - this->bbx_min[0])/this->dx+1;
    size_t npitch = (this->bbx_max[0] - this->bbx_min[0])/this->dx+1;

    size_t total = nx*ny*nz*nyaw*npitch+5;
    this->viewpoints.reserve(total);

    #pragma omp parallel
    {
        std::vector<Eigen::Matrix<double, 5, 1>> local_views;

        #pragma omp for nowait
        for (size_t ix = 0; ix <= nx; ++ix) {
            double x = this->bbx_min[0] + (double)ix * this->dx;
            for (double y = this->bbx_min[1]; y <= this->bbx_max[1]; y+= this->dy) {
                for (double z = this->bbx_min[2]; z <= this->bbx_max[2]; z += this->dz) {
                    for (double pitch = 0; pitch < M_PI; pitch+= dpitch) {
                        for (double yaw = 0; yaw < M_PI; yaw += dyaw) {
                            Eigen::Matrix<double,5,1> vp;
                            vp << x, y, z, pitch, yaw;
                            local_views.push_back(vp);
                        }
                    }
                }
            }
        }

        #pragma omp critical
        viewpoints.insert(viewpoints.end(), local_views.begin(), local_views.end());
    }
}

Eigen::Matrix4d nbvstrategy::getCameraPose(
    const Eigen::Matrix<double,5,1> &vp) 
{
    Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(
                            vp(3),Eigen::Vector3d::UnitY())
                            .toRotationMatrix();
    Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(
                            vp(4), Eigen::Vector3d::UnitZ())
                            .toRotationMatrix();
    Eigen::Matrix4d T;
    T.block<3,3>(0,0) = R_yaw*R_pitch;
    T.block<3,1>(0,3) = Eigen::Vector3d(vp(0),vp(1),vp(2)); 
    return T;
}

//for futher optimization - parallelable
open3d::geometry::PointCloud nbvstrategy::T_cam_pcd_to_world(
    const Eigen::Matrix4d &T_cam_world, 
    const open3d::geometry::PointCloud* pcd) 
{
    open3d::geometry::PointCloud pcd_world;
    pcd_world.points_.resize(pcd->points_.size());
    pcd_world.colors_.resize(pcd->colors_.size());
    
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(pcd->points_.size()); i++) {
        const auto &p = pcd->points_[i];
        Eigen::Vector4d p_homogeneous(p(0),p(1),p(2), 1.0);
        Eigen::Vector4d p_transformed_homogeneous = T_cam_world * p_homogeneous;
        pcd_world.points_[i] = p_transformed_homogeneous.head<3>();
        pcd_world.colors_[i] = pcd->colors_[i];
    }

    return pcd_world;
}

Eigen::Matrix4d nbvstrategy::
create_ellipsoid_dual_matrix(const EllipsoidParam &param){
    
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero(); 
    Eigen::Vector3d radii_pow = param.radii.array().square(); 
    Eigen::Vector3d radii_inv = radii_pow.array().inverse();
    Eigen::Matrix4d transformation = param.pose; 

    // 将radii_inv设置为matrix的前3x3的对角矩阵
    matrix.block<3,3>(0,0) = radii_inv.asDiagonal();
    matrix(3, 3) = -1;

    double det = matrix.determinant(); 
    if (det == 0) {
        std::cout << "The determinant of the matrix is 0, the matrix is not invertible." << std::endl;
        return Eigen::Matrix4d::Zero(); 
    } else {
        Eigen::Matrix4d matrix_dual_origin = matrix.inverse(); 
        // std::cout << "Matrix Dual Origin:\n" << matrix_dual_origin << std::endl;
        Eigen::Matrix4d matrix_dual = transformation * matrix_dual_origin * transformation.transpose(); // 计算最终的矩阵
        // std::cout << "Matrix Dual:\n" << matrix_dual << std::endl;
        return matrix_dual;
    }
}

bool SphereInFrustum(
        const Eigen::Vector3d &center_cam,
        double rx, double ry, double rz,
        const Eigen::Matrix3d &R_e_cam,
        const Camera &cam)
{
    // Compute bounding sphere in camera coordinates
    Eigen::Vector3d axis_x = R_e_cam.col(0) * rx;
    Eigen::Vector3d axis_y = R_e_cam.col(1) * ry;
    Eigen::Vector3d axis_z = R_e_cam.col(2) * rz;
    double R = std::max({axis_x.norm(), axis_y.norm(), axis_z.norm()});

    double x = center_cam.x();
    double y = center_cam.y();
    double z = center_cam.z();

    // Depth rejection
    if (z + R <= 0) return false;
    if (z - R > cam.max_range) return false;

    // Project center to pixels
    double u_center = cam.fx * (x / z) + cam.cx;
    double v_center = cam.fy * (y / z) + cam.cy;

    // Project sphere radius
    double u_rad = cam.fx * (R / z);
    double v_rad = cam.fy * (R / z);

    double u_min = u_center - u_rad;
    double u_max = u_center + u_rad;
    double v_min = v_center - v_rad;
    double v_max = v_center + v_rad;

    // Cull if completely outside image
    if (u_max < 0) return false;
    if (u_min >= cam.width) return false;
    if (v_max < 0) return false;
    if (v_min >= cam.height) return false;

    return true;
}

Eigen::Matrix3d nbvstrategy::
compute_ellipsoid_projection(
    const Eigen::Matrix<double, 3, 4> camera_matrix,
    const Eigen::Matrix4d ellipsoid_matrix_dual)
{
    

    Eigen::Matrix3d ellipse_dual = camera_matrix * 
                                    ellipsoid_matrix_dual * 
                                    camera_matrix.transpose();

    double det = ellipse_dual.determinant(); 
    if (det == 0) {
        std::cout << "The determinant of the matrix is 0, the matrix is not invertible." << std::endl;
        return Eigen::Matrix3d::Zero(); 
    } else {
        Eigen::Matrix3d ellipse = ellipse_dual.inverse();
        return ellipse;
    }

}

// also for further optimization
std::pair<double, cv::Mat> nbvstrategy::projectEllipsoidstoImage(
        const std::vector<EllipsoidParam> &ellipsoids,
        const Eigen::Matrix4d &T_cam_world) 
{
    // T_cam_world is expected to be the pose of the viewpoint
    Eigen::Matrix4d T_world_cam = T_cam_world.inverse();
    Eigen::Matrix3d R_wc = T_world_cam.block<3,3>(0,0);
    Eigen::Vector3d t_wc = T_world_cam.block<3,1>(0,3);
    
    // sanity reminder, don't need the camera matrix here yet since
    // transformations from 3d to 3d don't require intrinsic parameters
    // however projections for example 3d to 2d do required it 
    // so that will be done later
    std::vector<Eigen::Vector3d> centers(ellipsoids.size());
    #pragma omp parallel for
    for (size_t i = 0; i < ellipsoids.size(); i++) {
        Eigen::Vector3d center_world = ellipsoids[i].pose.block<3,1>(0,3);
        centers[i] = R_wc * center_world + t_wc; 
        // this will put it into the camera's world (e.g. if the camera was 0,0)
    }

    // weighting the centers, reminder that 
    // still don't need intrinsic parameters for this
    // since its already in the camera's world, we can use the z axis to weigh them
    std::vector<size_t> idx(centers.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(),
        [&centers](size_t i1, size_t i2) { return centers[i1].z() < centers[i2].z(); });

    // assigning the weights
    std::vector<double> weights(centers.size());
    for (size_t i = 0; i < idx.size(); i++) {
        weights[idx[i]] = 1 * pow(0.5, static_cast<double>(i));
    }
    
    // building camera matrix
    Eigen::Matrix3d K;
    K << cam.fx, 0,      cam.cx,
         0,      cam.fy, cam.cy,
         0,      0,      1;
    Eigen::Matrix3d R_inv = T_cam_world.block<3,3>(0,0).inverse();
    Eigen::Matrix<double,3,4> Rt_inv;
    Rt_inv.block<3,3>(0,0) = R_inv;
    Rt_inv.block<3,1>(0,3) = R_inv*T_cam_world.block<3,1>(0,3);
    Eigen::Matrix<double,3,4> P = K * Rt_inv;  

    // setting up image and ellipse vectors
    std::vector<double> a_vec(ellipsoids.size());
    std::vector<double> b_vec(ellipsoids.size());
    std::vector<double> c_vec(ellipsoids.size());
    std::vector<double> d_vec(ellipsoids.size());
    std::vector<double> e_vec(ellipsoids.size());
    std::vector<double> f_vec(ellipsoids.size());
    
    // projecting the ellipsoids
    // now this part will use the camera matrix
    // sanity reminder that the center transformations already put it in the camera world
    for (size_t k = 0; k < ellipsoids.size(); k++) {
        size_t i = idx[k];

        Eigen::Matrix3d R_e_world = ellipsoids[i].pose.block<3,3>(0,0);
        Eigen::Matrix3d R_e_cam = R_wc * R_e_world;
        
        double rx = ellipsoids[i].radii.x();
        double ry = ellipsoids[i].radii.y();
        double rz = ellipsoids[i].radii.z();
        
        //cull ellipsoids first too
        if (!SphereInFrustum(centers[i], rx, ry, rz, R_e_cam, cam)) {
            a_vec[i] = 0;
            b_vec[i] = 0;
            c_vec[i] = 0;
            d_vec[i] = 0;
            e_vec[i] = 0;
            f_vec[i] = 0;
            continue; // skip this ellipsoid
        }
        
        Eigen::Matrix4d dual = this->create_ellipsoid_dual_matrix(ellipsoids[i]);
        if (dual == Eigen::Matrix4d::Zero())
        {
            a_vec[i] = 0;
            b_vec[i] = 0;
            c_vec[i] = 0;
            d_vec[i] = 0;
            e_vec[i] = 0;
            f_vec[i] = 0;
            continue;
        }
        
        Eigen::Matrix3d ellipse_matrix = this->compute_ellipsoid_projection(P, dual);
        if (ellipse_matrix == Eigen::Matrix3d::Zero())
        {
            a_vec[i] = 0;
            b_vec[i] = 0;
            c_vec[i] = 0;
            d_vec[i] = 0;
            e_vec[i] = 0;
            f_vec[i] = 0;
            continue;
        }
        
        a_vec[i] = ellipse_matrix(0, 0);
        b_vec[i] = ellipse_matrix(1, 1);
        c_vec[i] = ellipse_matrix(0, 1) + ellipse_matrix(1, 0);
        d_vec[i] = ellipse_matrix(0, 2) + ellipse_matrix(2, 0);
        e_vec[i] = ellipse_matrix(1, 2) + ellipse_matrix(2, 1);
        f_vec[i] = ellipse_matrix(2, 2); 
    }
    
    // okay so we now have the centers, and the ellipse matrix coefficients
    // (a,b,c,d,e,f) for the ellipse equation
    // time to "project it"
    cv::Mat img(this->cam.height, this->cam.width, CV_8UC3, cv::Scalar(0,0,0));
    float occupied_res = 0;
    float frontier_res = 0;
    float roi_surface_frontier_res = 0;

    int totalPixels = this->cam.width * this->cam.height;

    for (int index = 0; index < totalPixels; index++) {
        int k = index / this->cam.width;
        int l = index % this->cam.width;
        double x = l;
        double y = k;
        double xx = x*x;
        double yy = y*y;
        double xy = x*y;

        for (size_t k = 0; k < ellipsoids.size(); k++) {
            size_t i = idx[k];

            if (a_vec[i] == 0 && b_vec[i] == 0 && c_vec[i] == 0 &&
                d_vec[i] == 0 && e_vec[i] == 0 && f_vec[i] == 0)
                continue;

            double value = a_vec[i]*xx + b_vec[i]*yy + c_vec[i]*xy + d_vec[i]*x + e_vec[i]*y + f_vec[i];
            if (value >= 0) continue;

            if (ellipsoids[i].type == "frontier") {
                img.at<cv::Vec3b>(k,l) = cv::Vec3b(0, 0, 255);  // Red
                frontier_res += 255 * weights[i];
            }
            else if (ellipsoids[i].type == "roi_surface_frontier") {
                img.at<cv::Vec3b>(k,l) = cv::Vec3b(0, 255, 0);  // Green
                roi_surface_frontier_res += 255 * weights[i];
            }
            else if (ellipsoids[i].type == "occupied") {
                img.at<cv::Vec3b>(k,l) = cv::Vec3b(255, 0, 0);  // Blue
                occupied_res += 255 * weights[i];
            }
        }
    }

    double score = 2*roi_surface_frontier_res + frontier_res - occupied_res;
    return std::make_pair(score, img);
}   

void nbvstrategy::getNBV(std::string ply_path, 
    double x, double y, double z, 
    double yaw,
    double pitch) 
{
    open3d::geometry::PointCloud pcd_camera;
    if (!open3d::io::ReadPointCloud(ply_path, pcd_camera)) {
        std::cerr << "Failed to read PLY file: " << ply_path << std::endl;
        return;
    }
    std::cout << "Loaded point cloud with " << pcd_camera.points_.size() << " points." << std::endl;

    // actually need to transform point cloud coordinates to world first
    // given its coordinates and pitch and yaw,
    // the equation for this would just be pitch matrix * yaw matrix
    // with the translation appended on the right, so would be a 4x4
    Eigen::Matrix4d camera_world = this->getCameraPose(Eigen::Matrix<double,5,1>(x,y,z,yaw,pitch));
    open3d::geometry::PointCloud pcd = this->T_cam_pcd_to_world(
        camera_world,
        &pcd_camera
    );
    Eigen::Vector3d coordinates = camera_world.block<3,1>(0,3);

    // crop point cloud
    this->voxel_struct->cropBBX(this->bbx_min, this->bbx_max, &pcd);
    std::cout << "\nPoint Cloud cropped!" << std::endl;
    std::cout << "New point cloud size: " << pcd.points_.size() << std::endl;

    // insert point cloud into voxelstruct
    this->voxel_struct->insertPointCloud(&pcd, coordinates);
    std::cout << "\nPoint Cloud inserted! " << std::endl;
    std::cout << "New size of Octree: " << this->voxel_struct->size(); 

    // classify voxels
    this->voxel_struct->classifyVoxels();

    // after fixing the voxels and preprocessing them, next is to do ellipsoid fitting
    std::vector<Eigen::Vector3d> surface_frontiers = this->voxel_struct->getSurfaceFrontiers();
    std::vector<Eigen::Vector3d> occupied_voxels = this->voxel_struct->getOccupiedVoxels();
    std::vector<Eigen::Vector3d> roi_surface_frontier = this->voxel_struct->getROISurfaceFrontiers();

    std::cout << "\nVoxels Classified!" << std::endl;
    std::cout << "Number of frontier voxels: " << surface_frontiers.size() << std::endl;
    std::cout << "Number of occupied voxels: " << occupied_voxels.size() << std::endl;
    std::cout << "Number of roi frontier voxels: " << roi_surface_frontier.size() << std::endl;

    // cluster each of these sets using gmm clusters
    std::vector<std::vector<Eigen::Vector3d>> frontier_clusters = this->ellipsoid_fitting->gmm_clustering(surface_frontiers);
    std::vector<std::vector<Eigen::Vector3d>> occupied_clusters = this->ellipsoid_fitting->gmm_clustering(occupied_voxels);
    std::vector<std::vector<Eigen::Vector3d>> roi_surface_frontier_clusters = this->ellipsoid_fitting->gmm_clustering(roi_surface_frontier);
    
    // ellipsoid fitting
    std::vector<EllipsoidParam> ellipsoids = this->ellipsoid_fitting->ellipsoidize_clusters_CGAL(
        frontier_clusters,
        occupied_clusters,
        roi_surface_frontier_clusters
    ); 

    // projection
    // get each vps pose and thats what you pass to project ellipsoids

}

