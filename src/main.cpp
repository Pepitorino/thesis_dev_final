#include <iostream>
#include "ellipsoid.hpp"
#include "nbvstrategy.hpp"
#include "voxelstruct.hpp"

using namespace std;

int main() {
    nbvstrategy nbv;
    while (true) {
        cout << "\n===== CSE1 THESIS MENU =====\n";
        cout << "1. Initialize\n";
        cout << "2. Get next best view\n";
        cout << "3. Reset next best view\n";
        cout << "4. Show last NBV score\n";
        cout << "5. Show last NBV image\n";
        cout << "6. Save NBV images\n";
        cout << "7. Save NBV scores\n";
        cout << "8. Exit\n";
        cout << "Enter option: ";

        int choice;
        cin >> choice;

        if (choice == 1) {
            {
                std::string sp;
                cout << "Enter settings path: ";
                cin >> sp;
                nbv.initialize(sp);
                cout << "\nNBV Initialized!" << std::endl;
            }
        }
        else if (choice == 2) {
            {  
                std::string fp;
                double x, y, z, yaw_deg, pitch_deg;
                cout << "Enter pcd file path: ";
                cin >> fp;
                cout << "Enter x,y,z,yaw,pitch: ";
                cin >> x >> y >> z >> yaw_deg >> pitch_deg;
                cout << std::endl;

                // turn yaw and pitch from degrees to radians:
                double yaw = yaw_deg * M_PI / 180.0;
                double pitch = pitch_deg * M_PI / 180.0;

                nbv.getNBV(fp, x, y, z, yaw, pitch);
            }
        }
        else if (choice == 3) {
            {
                // reset next best view
            }
        }
        else if (choice == 4) {
            {
                // show last nbv score
            }
        }
        else if (choice == 5) {
            {
                // show last nbv image
            }
        }
        else if (choice == 6) {
            {
                // save nbv images
            }
        }
        else if (choice == 7) {
            {
                // save nbv scores
            }
        }
        else if (choice == 8) {
            cout << "Exiting...\n";
            nbv.destroy();
            break;
        }
        else {
            cout << "Invalid option. Try again.\n";
        }
    }

    return 0;
}
