// author: xxb
// date: 2023.05.16

#include "calib_zhang.h"

// 输入: srcPoints: n*3 double 类型
// 输出: dstPoints: n*3 double 类型
// 输出: transMatrix: 变换矩阵
// 变换关系： dstPoints‘ = transMatrix * srcPoints'
//          此处srcPoitns'和dstPoints'均为 n*3 double类型，最后一列为1
void normalization(Eigen::MatrixXd &P, Eigen::Matrix3d &T)
{
    double cx = P.col(0).mean();
    double cy = P.col(1).mean();

    P.array().col(0) -= cx;
    P.array().col(1) -= cy;

    double num = (P.col(0).transpose() * P.col(0)).mean() + (P.col(1).transpose() * P.col(1)).mean();
    double stdx = sqrt(num) / sqrt(2 * P.rows());

    double scale = 1 / stdx;

    P.array().col(0) *= scale;
    P.array().col(1) *= scale;

    T << scale, 0, -scale * cx,
        0, scale, -scale * cy,
        0, 0, 1;
}

static Eigen::VectorXd solveHomographyDLT(Eigen::MatrixXd &srcNormal, Eigen::MatrixXd &dstNormal)
{

    int n = srcNormal.rows();
    // 2. DLT
    Eigen::MatrixXd input(2 * n, 9);

    for (int i = 0; i < n; ++i)
    {
        input(2 * i, 0) = 0.;
        input(2 * i, 1) = 0.;
        input(2 * i, 2) = 0.;
        input(2 * i, 3) = srcNormal(i, 0);
        input(2 * i, 4) = srcNormal(i, 1);
        input(2 * i, 5) = 1.;
        input(2 * i, 6) = -srcNormal(i, 0) * dstNormal(i, 1);
        input(2 * i, 7) = -srcNormal(i, 1) * dstNormal(i, 1);
        input(2 * i, 8) = -dstNormal(i, 1);

        input(2 * i + 1, 0) = srcNormal(i, 0);
        input(2 * i + 1, 1) = srcNormal(i, 1);
        input(2 * i + 1, 2) = 1.;
        input(2 * i + 1, 3) = 0.;
        input(2 * i + 1, 4) = 0.;
        input(2 * i + 1, 5) = 0.;
        input(2 * i + 1, 6) = -srcNormal(i, 0) * dstNormal(i, 0);
        input(2 * i + 1, 7) = -srcNormal(i, 1) * dstNormal(i, 0);
        input(2 * i + 1, 8) = -dstNormal(i, 0);
    }

    // 3. SVD分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svdSolver(input, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd V = svdSolver.matrixV();

    return V.rightCols(1);
}

#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct HOMOGRAPHY_COST {

    HOMOGRAPHY_COST(double x1, double y1,double x2, double y2)
        :x1(x1), y1(y1), x2(x2), y2(y2)
    {}

    template<typename T>
    bool operator()(const T *const h, T* residual) const
    {

        T w = T(h[6]*x1+h[7]*y1+h[8]);
        T x = T(h[0]*x1+h[1]*y1+h[2])/w;
        T y = T(h[3]*x1+h[4]*y1+h[5])/w;

        residual[0] = ceres::sqrt(ceres::pow(T(x2)-x, 2) + ceres::pow(T(y2)-y, 2));

        return true;
    }

    const double x1, x2, y1, y2;

};

Eigen::Matrix3d calib_zhang::LMOptimization(std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints, Eigen::Matrix3d &H)
{
    H.transposeInPlace();
    Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd>(H.data(), H.size()); // 将矩阵按列展开成一个9x1的向量

    int n = srcPoints.size();

    ceres::Problem optimizationProblem;
    for(int i=0; i<n; ++i)
    {
        optimizationProblem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<HOMOGRAPHY_COST, 1, 9>(
                new HOMOGRAPHY_COST(srcPoints[i].x, srcPoints[i].y, dstPoints[i].x, dstPoints[i].y)
            ),
            nullptr,
            v.data()
        );
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &optimizationProblem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    // H矩阵转置回去
    H.transposeInPlace();

    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> H1(v.data());
    return H1;
}

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

    normalization(srcNormal, srcT);
    normalization(dstNormal, dstT);

    // DLT计算特征向量
    Eigen::VectorXd v = solveHomographyDLT(srcNormal, dstNormal);

    Eigen::Matrix3d M;
    M << v(0), v(1), v(2),
        v(3), v(4), v(5),
        v(6), v(7), v(8);

    // LM优化
    Eigen::Matrix3d M1 = LMOptimization(srcPoints, dstPoints, M);

    // 4. 反计算H
    HMartix = dstT.inverse() * M1 * srcT;
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
        srcPointsMatrix(j, 2) = 1.0;

        dstPointsMatrix(j, 0) = dstPoints[j].x;
        dstPointsMatrix(j, 1) = dstPoints[j].y;
        dstPointsMatrix(j, 2) = 1.0;
    }

    // 计算dst=H*src
    Eigen::MatrixXd dstPointsMatrixResult(nSize, 3);
    dstPointsMatrixResult = H * srcPointsMatrix.transpose();

    // 第一列和第二列除以第三列
    dstPointsMatrixResult.row(0) = dstPointsMatrixResult.row(0).cwiseQuotient(dstPointsMatrixResult.row(2));
    dstPointsMatrixResult.row(1) = dstPointsMatrixResult.row(1).cwiseQuotient(dstPointsMatrixResult.row(2));
    // std::cout << dstPointsMatrixResult << std::endl;

    // 结果相减
    Eigen::MatrixXd dstPointsMatrixResual(nSize, 3);
    dstPointsMatrixResual = dstPointsMatrixResult.transpose() - dstPointsMatrix;

    // std::cout << dstPointsMatrixResual << std::endl;

    // 计算残差
    double res = dstPointsMatrixResual.col(0).squaredNorm() + dstPointsMatrixResual.col(1).squaredNorm();

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
    for (int j = 0; j < m_filesNum; j++)
    {
        printf("\n\ncompute the [%d]th image!\n", j + 1);
        Eigen::Matrix3d H;
        Eigen::Matrix3d HOpencv;

        H = findHomography(m_objectPoints[j], m_imagePoints[j]);
        // Eigen::Matrix3d H1 = LMOptimization(m_objectPoints[j], m_imagePoints[j], H);
        double resualSelf = computeResudual(m_objectPoints[j], m_imagePoints[j], H);

        // 使用OpenCV函数来计算H矩阵
        cv::Mat hMat = cv::findHomography(m_objectPoints[j], m_imagePoints[j], cv::LMEDS);
        // mat到egien的转换
        cv::cv2eigen(hMat, HOpencv);
        double resualOpenCV = computeResudual(m_objectPoints[j], m_imagePoints[j], HOpencv);

        // 使用colmap库来计算H矩阵
        // HomographyMatrixEstimator est_tform;

        // std::vector<Eigen::Vector2d> objectPoints;
        // convertPoint2fToVector2d(m_objectPoints[j], objectPoints);
        // std::vector<Eigen::Vector2d> imagePoints;
        // convertPoint2fToVector2d(m_imagePoints[j], imagePoints);

        // std::vector<Eigen::Matrix3d> models = est_tform.Estimate(objectPoints, imagePoints);
        // double resualcolmap = computeResudual(m_objectPoints[j], m_imagePoints[j], models[0]);

        // est_tform.Residuals(objectPoints, imagePoints, models[0]);

        std::cout << "show resual:" << std::endl;
        std::cout << H << std::endl;
        std::cout << "resual self: " << resualSelf << std::endl;
        std::cout << HOpencv << std::endl;
        std::cout << "resual opencv: " << resualOpenCV << std::endl;
        // std::cout << models[0] << std::endl;
        // std::cout << "resual colmap: " << resualcolmap << std::endl;

        m_homos.push_back(H);
    }
    std::cout << " fit homography finished\n\n"
              << std::endl;
}