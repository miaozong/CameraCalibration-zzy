// author: xxb
// date: 2023.05.15

#ifndef _CALIB_IMAGE_IO_H
#define _CALIB_IMAGE_IO_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include <opencv2/opencv.hpp>

#include <iostream>
using namespace std;
using namespace cv;

class calib_image_io
{

public:
    // 加载图像，并对加载的图像进行角点检测，存入成员变量中
    int loadImageAndFindChessboardCorners(std::vector<std::string> &files, int board_h, int board_w, int board_size);

private:
    // 打印坐标信息
    void printCoordinates();

    // 图像文件所在的全路径
    std::vector<std::string> m_files;

    // 真实载入的图像数目
    int m_filesNum;

    // 棋盘格相关:
    int m_board_h;         // 棋盘格大小，例如9*6。此处即为9
    int m_board_w;         // 棋盘格大小，例如9*6，此处即为6
    int m_boardSquareSize; // 单位：mm  eg: 50mm 25mm

public:
    // 保存所有角点的图像坐标
    std::vector<std::vector<cv::Point2f>> m_imagePoints;

    // 棋盘格上角点的三维坐标
    std::vector<std::vector<cv::Point2f>> m_objectPoints;
};

#endif