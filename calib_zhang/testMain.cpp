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

    calib_imageio.loadImageAndFindChessboardCorners(files, 9, 6, 25);

    // 开始标定
    calib_zhang calib_algorithm;
    calib_algorithm.calibrateCamera(calib_imageio.m_imagePoints, calib_imageio.m_objectPoints);

    return 0;
}