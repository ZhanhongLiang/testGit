/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-10-23 22:03:21
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-19 21:32:35
 */

// TODO角度转换的实现
#include "../../include/rune/AngleSolver.hpp"

namespace detect {

namespace angle_solver {

#ifdef MINECODE
//初始化作用

/**
 * @brief:
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */
void AngleSolver::GetRuneAngle() {
  //返回rvec,tvec;
  //用返回的rect找值
  // vector<Point2f> targetP2P;
  // GetP2Point(rect, targetP2P, Point2f(0, 0));
  // imagePoints = targetP2P;
  solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  std::cout << "tvec" << tvec.at<double>(2, 0) << std::endl;
  // std::cout << "rvec:" << rvec.at<double>(2,0) << std::endl;
  // std::cout << "tvec:" << tvec << std::endl;
}

/**
 * @brief: 获得装甲板的世界坐标，两个已知条件中一个条件
 * @param uint mode
 * @param Point3f offset_point//补偿点值
 * @return:
 * @author: Chinwong_Leung
 */
void AngleSolver::GetP3Point(uint mode, Point2f offset_point) {
  objectPoints.clear();
  double half_x, half_y;

  half_x = flag->flagArmorParam.flagArmorParamWidth / 2.0;
  half_y = flag->flagArmorParam.flagArmorParamHeight / 2.0;
  objectPoints.push_back(Point3f(-half_x, -half_y, 0) +
                         Point3f(offset_point.x, offset_point.y, 0));
  objectPoints.push_back(Point3f(half_x, -half_y, 0) +
                         Point3f(offset_point.x, offset_point.y, 0));
  objectPoints.push_back(Point3f(half_x, half_y, 0) +
                         Point3f(offset_point.x, offset_point.y, 0));
  objectPoints.push_back(Point3f(-half_x, half_y, 0) +
                         Point3f(offset_point.x, offset_point.y, 0));

  std::cout << "objectPoints1:" << objectPoints[0] << std::endl;
  std::cout << "objectPoints1:" << objectPoints[1] << std::endl;
  std::cout << "objectPoints1:" << objectPoints[2] << std::endl;
  std::cout << "objectPoints1:" << objectPoints[3] << std::endl;
}

/**
 * @brief: 获得图像的像素点值,两个已知条件中一个条件
 * @param {type}
 * @return: target2D
 * @return:
 * @author: Chinwong_Leung
 */

void AngleSolver::GetP2Point(RotatedRect &rect, vector<cv::Point2f> &target2D,
                             Point2f offset_point) {
  Point2f lu, ld, ru, rd;
  Point2f vertices[4];
  rect.points(vertices);
  sort(vertices, vertices + 4,
       [](const Point2f &p1, const Point2f &p2) { return p1.x < p2.x; });
  //排序
  if (vertices[0].y > vertices[1].y) {
    lu = vertices[1];
    ld = vertices[0];
  } else if (vertices[0].y < vertices[1].y) {
    lu = vertices[0];
    ld = vertices[1];
  } else if (vertices[2].y > vertices[3].y) {
    ru = vertices[3];
    rd = vertices[2];
  } else if (vertices[2].y < vertices[3].y) {
    ru = vertices[2];
    rd = vertices[3];
  }
  target2D.clear();
  target2D.push_back(lu + offset_point);
  target2D.push_back(ru + offset_point);
  target2D.push_back(ld + offset_point);
  target2D.push_back(rd + offset_point);
}

/**
 * @brief:
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */
void AngleSolver::FourPointSolvePnP(RotatedRect &rect_, float &angle_x,
                                    float &angle_y, float &dist) {
  float X, Z, Y;
  float yCorrected;
  GetP2Point(rect_, imagePoints, Point2f(0, 0));
  solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  X = tvec.at<float>(0, 0);
  Y = tvec.at<float>(1, 0);
  Z = tvec.at<float>(2, 0);
  tvec.at<float>(1, 0) -= ptz_camera_z;
  //对摄像头到枪管进行修正
  angle_x = atan(tvec.at<float>(0, 0) / tvec.at<float>(2, 0) * CV_PI / 180);
  angle_y = atan(tvec.at<float>(1, 0) / tvec.at<float>(2, 0) * CV_PI / 180);
  // dist = tvec.at<float>(2, 0);
  dist = sqrt(tvec.at<float>(0, 0) * tvec.at<float>(0, 0) +
              tvec.at<float>(1, 0) * tvec.at<float>(1, 0) +
              tvec.at<float>(2, 0) * tvec.at<float>(2, 0));
  std::cout << "angle_x: " << angle_x << std::endl;
  std::cout << "angle_y: " << angle_y << std::endl;
  std::cout << "Z: " << tvec.at<float>(2, 0) << std::endl;
  //对枪管到云台进行修正
}
/**
 * @brief:
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */

// float AngleSolver::GetRunePitch(float dist, float tvecy_y, float
// ballet_speed)
// {
//      //
//      申明临时y轴方向长度,子弹实际落点，实际落点与击打点三个变量不断更新（mm）
//     float y_temp, y_actual, dy;
//     // 重力补偿枪口抬升角度
//     float a = 0.0;
//     float GRAVITY = 9.7887f; //shenzhen 9.7887  zhuhai
//     y_temp = tvec_y;
//     // 迭代求抬升高度
//     for (int i = 0; i < 10; i++) {
//         // 计算枪口抬升角度
//         a = (float) atan2(y_temp, dist);
//         // 计算实际落点
//         float t, y = 0.0;
//         t = dist / (ballet_speed * cos(a));
//         y_actual = ballet_speed * sin(a) * t - GRAVITY * t * t / 2;
//         dy = tvec_y - y_actual;
//         y_temp = y_temp + dy;
//         // 当枪口抬升角度与实际落点误差较小时退出
//         if (fabsf(dy) < 0.01) {
//             break;
//         }
//     }
//     return a;
// }
/**
 * @brief:
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */
void AngleSolver::GetCamerParam() {
  Settings &settings = *settings_;
  FileStorage fs(settings.intrinsic_file_480, FileStorage::READ);
  fs["Camera_Matrix"] >> cameraMatrix;
  fs["Distortion_Coefficients"] >> distCoeffs;
  std::cout << "cameraMatrix:" << cameraMatrix << std::endl;
  std::cout << "distCoeffs:" << distCoeffs << std::endl;
  fs.release();
}

/**
 * @brief:单点解算pnp，参照的是东南大学和learning opencv的畸变矫正
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */
void AngleSolver::OnePointSolvePnP(Point2f &armorCenter, float &angle_x,
                                   float &angle_y) {
  float xScreen, yScreen, fX, fY, cX, cY;
  float angleX, angleY;
  float angleXCorrected, angleYCorrected;
  vector<Point2f> uncorrectedPoint;
  vector<Point2f> correctedPoint;

  Point2f processPoint;

  xScreen = armorCenter.x;
  yScreen = armorCenter.y;
  std::cout << "xScreen: " << xScreen << std::endl;
  std::cout << "yScreen: " << yScreen << std::endl;
  cX = cameraMatrix.at<float>(0, 2);
  cY = cameraMatrix.at<float>(1, 2);
  fX = cameraMatrix.at<float>(0, 0);
  fY = cameraMatrix.at<float>(1, 1);
  uncorrectedPoint.push_back(armorCenter);

  undistortPoints(uncorrectedPoint, correctedPoint, cameraMatrix, distCoeffs,
                  noArray(), cameraMatrix);
  // std::cout << "correctedPoint:" << correctedPoint[0] << std::endl;
  processPoint = correctedPoint.front();

  //未修正的角度
  angleX = (xScreen - cX) / fX;
  angleY = (yScreen - cY) / fY;

  //修正的角度
  angleXCorrected = (processPoint.x - cX) / fX;
  angleYCorrected = (processPoint.y - cY) / fY;

  std::cout << "angleXcorrected: " << angleXCorrected << std::endl;
  std::cout << "angleYcorrected: " << angleYCorrected << std::endl;
  //泰勒级数展开的修正，畸变修正
  angle_x = atan(angleXCorrected) / CV_PI * 180;
  angle_y = atan(angleYCorrected) / CV_PI * 180;
  std::cout << "angle_x" << angle_x << std::endl;
  std::cout << "angle_y" << angle_y << std::endl;
}

#endif

//官方代码修改
#ifndef MINECODE
#endif
}  // namespace angle_solver
}  // namespace detect
