#include <iostream>
#include "ellipsoid.h"
#include "nbvstrategy.h"
#include "voxelstruct.h"

using namespace std;

int main() {
    nbvstrategy nbv();
    while (true) {
        cout << "\n===== CSE1 THESIS MENU =====\n";
        cout << "1. Get next best view\n";
        cout << "2. Reset next best view\n";
        cout << "3. Show last NBV score\n";
        cout << "4. Show last NBV image\n";
        cout << "5. Save NBV images\n";
        cout << "6. Save NBV scores\n";
        cout << "7. Exit\n";
        cout << "Enter option: ";

        int choice;
        cin >> choice;

        if (choice == 1) {
            {
                // get next best view
            }
        }
        else if (choice == 2) {
            {
                // reset next best view
            }
        }
        else if (choice == 3) {
            {
                // show last nbv score
            }
        }
        else if (choice == 4) {
            {
                // show last nbv image
            }
        }
        else if (choice == 5) {
            {
                // show nbv images
            }
        }
        else if (choice == 6) {
            {
                // show nbv scores
            }
        }
        else if (choice == 7) {
            cout << "Exiting...\n";
            break;
        }
        else {
            cout << "Invalid option. Try again.\n";
        }
    }

    return 0;
}
