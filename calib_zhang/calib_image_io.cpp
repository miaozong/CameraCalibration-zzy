// author: xxb
// date: 2023.05.15

#include "calib_image_io.h"

void calib_image_io::printCoordinates()
{
    // 打印图像信息
    printf("image number is [%d]\n", m_filesNum);
    printf("image full path:\n");
    for (int j = 0; j < m_files.size(); j++)
    {
        printf("[%d] %s\n", j + 1, m_files[j].c_str());
    }

    // 打印坐标信息
    for (int j = 0; j < m_imagePoints.size(); j++)
    {
        printf("show the [%d]th image points:\n", j + 1);
        for (int i = 0; i < m_imagePoints[j].size(); i++)
        {
            printf("[%f] [%f]\n", m_imagePoints[j][i].x, m_imagePoints[j][i].y);
        }

        printf("show the [%d]th object points:\n", j + 1);
        for (int i = 0; i < m_objectPoints[j].size(); i++)
        {
            printf("[%f] [%f]\n", m_objectPoints[j][i].x, m_objectPoints[j][i].y);
        }
    }
}

int calib_image_io::loadImageAndFindChessboardCorners(std::vector<std::string> &files, int board_h, int board_w, int board_size)
{
    // 初始化成员变量
    m_filesNum = files.size();
    m_files.assign(files.begin(), files.end());
    m_board_h = board_h;
    m_board_w = board_w;
    m_boardSquareSize = board_size;

    // 棋盘格大小
    cv::Size boardSize(board_h, board_w);

    // 单元格大小， 单位mm
    float fBoardSize = board_size * 1.0f;
    cv::Size2f squareSize(fBoardSize, fBoardSize);

    for (int i = 0; i < files.size(); ++i)
    {
        // 读取源图
        cv::Mat img = cv::imread(m_files[i]);
        std::vector<cv::Point2f> corners;

        // 转换灰度图像
        cv::Mat gray;
        cv::cvtColor(img, gray, CV_BGR2GRAY);

        // 提取角点
        bool found_success = cv::findChessboardCorners(img, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found_success)
        {
            // printf("before cornerSubPix:\n");
            // for(int i = 0; i < corners.size(); i++){
            //     printf("[%f] [%f]\n",corners[i].x, corners[i].y);
            // }

            // 亚像素检测
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));

            // printf("after cornerSubPix:\n");
            // for(int i = 0; i < corners.size(); i++){
            //     printf("[%f] [%f]\n",corners[i].x, corners[i].y);
            // }

            // 显示角点
            // cv::drawChessboardCorners(img, boardSize, cv::Mat(corners), found_success);
            // cv::imshow("after", img);
            // cv::waitKey(200000);

            // 保存角点坐标
            m_imagePoints.push_back(corners);
        }
    }

    // 设置世界坐标
    for (int i = 0; i < m_imagePoints.size(); ++i)
    {
        std::vector<cv::Point2f> corners;

        // 以棋盘格左上角为原点
        for (int j = 1; j <= m_board_h; j++)
        {
            for (int i = 1; i <= m_board_w; i++)
            {
                corners.push_back(cv::Point2f(i * m_boardSquareSize, j * m_boardSquareSize));
            }
        }

        m_objectPoints.push_back(corners);
    }

    // 打印坐标信息
    // printCoordinates();

    return 0;
}