// 输入: srcPoints: n*3 double 类型
// 输出: dstPoints: n*3 double 类型
// 输出: transMatrix: 变换矩阵
// 变换关系： dstPoints‘ = transMatrix * srcPoints'  
//          此处srcPoitns'和dstPoints'均为 n*3 double类型，最后一列为1
void normalization(Eigen::MatrixXd &srcPoints, Eigen::MatrixXd &dstPoints, Eigen::Matrix3d &transMatrix)
{
    double cx = srcPoints.col(0).mean();
    double cy = srcPoints.col(1).mean();

    P.array().col(0) -= cx;
    P.array().col(1) -= cy;

    double num = (P.col(0).transpose() * P.col(0)).mean() + (P.col(1).transpose() * P.col(1)).mean();
    double stdx = sqrt(num) / sqrt(2*P.rows());


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