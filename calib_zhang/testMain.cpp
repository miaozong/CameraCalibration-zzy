#include "calib_image_io.h"
#include "calib_zhang.h"

using namespace Eigen;
using namespace std;
int main(int argc, char **argv) {

    calib_image_io calib_imageio;
    std::vector<std::string> files = {
        "../../data/images/left01.jpg",
        "../../data/images/left02.jpg",
        "../../data/images/left03.jpg",
        "../../data/images/left04.jpg",
        "../../data/images/left05.jpg",
        "../../data/images/left06.jpg",
        "../../data/images/left07.jpg",
        "../../data/images/left08.jpg",
        "../../data/images/left09.jpg",
        "../../data/images/left11.jpg",
        "../../data/images/left12.jpg",
        "../../data/images/left13.jpg",
        "../../data/images/left14.jpg",
    };

    // calib_imageio.loadImageAndFindChessboardCorners(files, 9, 6, 25);

    // 开始标定
    // calib_zhang calib_algorithm;
    // calib_algorithm.calibrateCamera(calib_imageio.m_imagePoints, calib_imageio.m_objectPoints);

    MatrixXd mat(3, 3);
    mat << 1, 2, 3,
           4, 5, 6,
           7, 8, 9;

    MatrixXd mat2 = mat.col(0).array() / mat.col(2).array(); // 将第一列元素除以第三列元素

    std::cout << mat2 << std::endl;

    return 0;
    
    
    return 0;
}