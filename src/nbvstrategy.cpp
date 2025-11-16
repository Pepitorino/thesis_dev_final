#include "nbvstrategy.hpp"
#include "json.hpp"

nbvstrategy::nbvstrategy() {

}

template<typename T>
T getOrDefault(const json& j, const std::string& key, T fallback) {
    if (j.contains(key) && !j[key].is_null()) {
        return j[key].get<T>();
    }
    return fallback;
}

std::vector<double> getVectorOrEmpty(const json& j, const std::string& key) {
    if (j.contains(key) && j[key].is_array())
        return j[key].get<std::vector<double>>();
    return {};
}

Eigen::Vector3d getVec3OrDefault(const json& j, const std::string& key) {
    if (j.contains(key)) {
        const auto& obj = j[key];
        if (obj.contains("x") && obj.contains("y") && obj.contains("z")) {
            return Eigen::Vector3d(obj["x"], obj["y"], obj["z"]);
        }
    }
    return Eigen::Vector3d::Zero();
}

int nbvstrategy::initialize() {
    std::ifstream file("settings.json");
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
    double max_range = getOrDefault<double>(cam, "max_range", 0);

    // loads global bbx
    auto bbx = cfg["bounding_box"];
    std::vector<double> bbx_min = getVectorOrEmpty(bbx, "bbx_min");
    std::vector<double> bbx_max = getVectorOrEmpty(bbx, "bbx_max");

    // loads plant bbxs
    std::vector<PlantBBX> plant_bbx_list;
    auto plants_region = cfg["plants_region"];
    for (auto& p : plants_region["plants"]) {
        PlantBBX pb;
        pb.min = getVec3OrDefault(p, "min");
        pb.max = getVec3OrDefault(p, "max");
        plant_bbx_list.push_back(pb);
    }
    
    // loads cluster numbers;
    auto clustering = cfg["clustering"];
    int min_clusters = getOrDefault(clustering, "min_clusters", 2);
    int max_clusters = getOrDefault(clustering, "max_clusters", 10);

    // loads octomap res
    auto octomap = cfg["octomap"];
    double resolution = getOrDefault(octomap, "resolution",  0.05);

    // loads viewpoint generation frequency
    auto xyzypgenf = cfg["xyzypgenf"];
    double dx = getOrDefault(xyzypgenf, "dx", 0.05);
    double dy = getOrDefault(xyzypgenf, "dy", 0.05);
    double dz = getOrDefault(xyzypgenf, "dz", 0.05);
    double dyaw = getOrDefault(xyzypgenf, "dyaw", 8);
    double dpitch = getOrDefault(xyzypgenf, "dpitch", 8);

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
    this->ellipsoid_fitting = new ellipsoid_fitting(this->min_clusters, this->max_clusters);

    this->generateViewpoints();
    return 0;
}

void nbvstrategy::generateViewpoints() {
    size_t nx = (this->bbx_max[0] - this->bbx_min[0])/this->dx+1;
    size_t ny = (this->bbx_max[0] - this->bbx_min[0])/this->dx+1;
    size_t nz = (this->bbx_max[0] - this->bbx_min[0])/this->dx+1;
    size_t nyaw = (this->bbx_max[0] - this->bbx_min[0])/this->dx+1;
    size_t npitch = (this->bbx_max[0] - this->bbx_min[0])/this->dx+1;

    size_t total = nx*ny*nz*nyaw*npitch+5;
    this->viewpoints,reserve(total);

    #pragma omp parallel
    {
        std::vector<Eigen::Matrix<double, 5, 1>> local_views;

        #pragma omp for nowait
        for (double x = this->bbx_min[0]; x <= this->bbx_max[0]; x += this->dx) {
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