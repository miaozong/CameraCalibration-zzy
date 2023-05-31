// author: xxb
// date: 2023.05.16
// description: zhang's calibration method.

#ifndef _CALIB_ZHANG_H
#define _CALIB_ZHANG_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>
// using namespace std;
// using namespace cv;

class calib_zhang
{
public:
    // 计算单张图像的单应性矩阵H
    Eigen::Matrix3d findHomography(std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints);

    // LM优化
    Eigen::Matrix3d LMOptimization(std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints, Eigen::Matrix3d& H);

    // 计算单张图像H矩阵的残差来验证计算的有效性,返回值为输出结果
    double computeResudual(std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints, Eigen::Matrix3d &H);

    void calibrateCamera(std::vector<std::vector<cv::Point2f>> &imagePoints, std::vector<std::vector<cv::Point2f>> &objectPoints);

private:
    

    // 每张图片计算的单应性矩阵
    std::vector<Eigen::Matrix3d> m_homos;

    // 真实载入的图像数目
    int m_filesNum;

    // 保存所有角点的图像坐标
    std::vector<std::vector<cv::Point2f>> m_imagePoints;

    // 棋盘格上角点的三维坐标
    std::vector<std::vector<cv::Point2f>> m_objectPoints;
};

#endif