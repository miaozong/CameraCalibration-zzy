// author: xxb
// date: 2023.05.16

#include "calib_zhang.h"

Eigen::Matrix3d calib_zhang::findHomography(std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
{
    // 单应性矩阵
    Eigen::Matrix3d HMartix;

    // 输入的像素点数目应一一对应
    if (srcPoints.size() != dstPoints.size())
    {
        printf("findHomography size dismatch!\n");
    }

    // 构造n*3 double矩阵
    int nSize = srcPoints.size();
    Eigen::MatrixXd srcNormal(nSize, 3);
    Eigen::MatrixXd dstNormal(nSize, 3);
    for (int j = 0; j < nSize; j++)
    {
        srcNormal(j, 0) = srcPoints[j].x;
        srcNormal(j, 1) = srcPoints[j].y;
        srcNormal(j, 2) = 1.0f;

        dstNormal(j, 0) = dstPoints[j].x;
        dstNormal(j, 1) = dstPoints[j].y;
        dstNormal(j, 2) = 1.0f;
    }

    // 归一化
    Eigen::Matrix3d srcT, dstT;
    
    // normalization(srcNormal, srcT);
    // normalization(dstNormal, dstT);

    // DLT计算特征向量
    // Eigen::VectorXd v = solveHomographyDLT(srcNormal, dstNormal);
    Eigen::VectorXd v;

    Eigen::Matrix3d M;
    M << v(0), v(1), v(2),
        v(3), v(4), v(5),
        v(6), v(7), v(8);

    // 4. 反计算H
    HMartix = dstT.inverse() * M * srcT;
    HMartix.array() /= HMartix(8);

    return HMartix;
}

double calib_zhang::computeResudual(std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints, Eigen::Matrix3d &H)
{
    // 构造n*3 double矩阵
    int nSize = srcPoints.size();
    Eigen::MatrixXd srcPointsMatrix(nSize, 3);
    Eigen::MatrixXd dstPointsMatrix(nSize, 3);
    for (int j = 0; j < nSize; j++)
    {
        srcPointsMatrix(j, 0) = srcPoints[j].x;
        srcPointsMatrix(j, 1) = srcPoints[j].y;
        srcPointsMatrix(j, 2) = 1.0f;

        dstPointsMatrix(j, 0) = dstPoints[j].x;
        dstPointsMatrix(j, 1) = dstPoints[j].y;
        dstPointsMatrix(j, 2) = 1.0f;
    }

    // 计算dst=H*src
    Eigen::MatrixXd dstPointsMatrixResult(nSize, 3);
    dstPointsMatrixResult = H * srcPointsMatrix.transpose();

    // std::cout << dstPointsMatrixResult << std::endl;

    // 结果相减
    Eigen::MatrixXd dstPointsMatrixResual(nSize, 3);
    dstPointsMatrixResual = dstPointsMatrixResult.transpose() - dstPointsMatrix;

    std::cout << dstPointsMatrixResual << std::endl;

    double res = dstPointsMatrixResual.squaredNorm();

    return res;
}

void convertPoint2fToVector2d(std::vector<cv::Point2f> &src, std::vector<Eigen::Vector2d> &dst)
{
    for (int j = 0; j < src.size(); j++)
    {
        cv::Point2f pointSrc = src[j];
        Eigen::Vector2d pointDst(pointSrc.x, pointSrc.y);
        dst.push_back(pointDst);
    }
}

#include "homography_matrix.h"
void calib_zhang::calibrateCamera(std::vector<std::vector<cv::Point2f>> &imagePoints, std::vector<std::vector<cv::Point2f>> &objectPoints)
{
    // 将输入坐标存入成员变量中
    m_imagePoints.assign(imagePoints.begin(), imagePoints.end());
    m_objectPoints.assign(objectPoints.begin(), objectPoints.end());

    // 输入的图像数目应一一对应
    if (m_imagePoints.size() != m_objectPoints.size())
    {
        printf("calibrateCamera size dismatch!\n");
        return;
    }

    // 计算H矩阵
    printf(" fit homography .....\n");
    m_filesNum = m_imagePoints.size();

    // 遍历每一张图片，计算H
    for (int j = 0; j < 1; j++)
    {
        printf("compute the [%d]th image!\n", j + 1);
        Eigen::Matrix3d H;
        Eigen::Matrix3d HOpencv;

        // H = findHomography(m_objectPoints[j], m_imagePoints[j]);
        // double resualSelf = computeResudual(m_objectPoints[j], m_imagePoints[j], H);

        // 使用OpenCV函数来计算H矩阵
        cv::Mat hMat = cv::findHomography(m_objectPoints[j], m_imagePoints[j], 0);

        // mat到egien的转换
        cv::cv2eigen(hMat, HOpencv);
        double resualOpenCV = computeResudual(m_objectPoints[j], m_imagePoints[j], HOpencv);

        // 使用colmap库来计算H矩阵
        HomographyMatrixEstimator est_tform;

        std::vector<Eigen::Vector2d> objectPoints;
        convertPoint2fToVector2d(m_objectPoints[j], objectPoints);
        std::vector<Eigen::Vector2d> imagePoints;
        convertPoint2fToVector2d(m_imagePoints[j], imagePoints);

        std::vector<Eigen::Matrix3d> models = est_tform.Estimate(objectPoints, imagePoints);
        double resualcolmap = computeResudual(m_objectPoints[j], m_imagePoints[j], models[0]);

        std::cout << std::endl << "show resual:" << std::endl;
        // std::cout << H << std::endl;
        // std::cout << "resual self: " << resualSelf << std::endl;
        std::cout << HOpencv << std::endl;
        std::cout << "resual opencv: " << resualOpenCV << std::endl;
        std::cout << models[0] << std::endl;
        std::cout << "resual colmap: " << resualcolmap << std::endl;

        m_homos.push_back(H);
    }
    std::cout << " fit homography finished" << std::endl;
}