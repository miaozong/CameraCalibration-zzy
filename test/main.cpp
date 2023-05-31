#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
    Eigen::VectorXd vec(9);
    vec << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mat(vec.data());
    std::cout << mat << std::endl;

    return 0;
}