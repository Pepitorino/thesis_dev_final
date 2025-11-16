#include <iostream>
#include "ellipsoid.hpp"
#include "nbvstrategy.hpp"
#include "voxelstruct.hpp"

using namespace std;

int main() {
    nbvstrategy nbv;
    voxelstruct voxelstruct(0.05);
    ellipsoid ellipse;
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
                nbv.initialize();
            }
        }
        else if (choice == 2) {
            {  
                cout << "Enter next file path: ";
                cin >> fp;
                
                // get next best view
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
