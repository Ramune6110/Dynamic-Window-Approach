#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include "dwa.h"

using namespace std;
using namespace Eigen;

int main() 
{
    Dynamic_Window_Approach DWA;

    DWA.simulation();

    return 0;
}
