#include "ellipsoid.hpp"
#include "structs.hpp"
#include "nbvstrategy.hpp"
#include "voxelstruct.hpp"
#include <string>


int main (int argc, char** argv) {
    if (argc < 2) {
        printf("Error: no path to settings");
        return -1;
    }
    std::string settings_path = std::string(argv[1]);
    voxelstruct voxelstruct(0.05);
    nbvstrategy nbv;
    if(nbv.initialize(settings_path)==-1) return -1;
    return 0;
} 