/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-11-26 20:36:15
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-26 21:48:01
 */
#include <math.h>
#include <omp.h>
#include <time.h>
#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

//#define DEBUG
//#define DEBUG1
#define Windows
//#define Linux_EXPOSURE

enum { BLUE, RED };
extern int teamSelect;
extern double result;

class Armor_Detector {
 public:
  int blue_color_diff = 70;
  int red_color_diff = 50;
  int light_threshold_val = 200;
  int count = 0;
  int fd;
  float yaw, pitch;
  int targetGet = 0;
  int angleFalse = 0;
  int angleCount = 0;
  int angleZero = 0;
  int minRatio = 1.1;
  Point2f angles[5];
  Point2f nowAngle, nextAngle;
  Mat src, pretreatDst;
  vector<RotatedRect> ArmorBuild, lights;
  RotatedRect targetArmor, lastTargetarmor;  //�ڱ�������Ŀ��װ�װ�
  // float pitch, yaw;

  //~ Armor_Detector(int fdCar) {
  //~ fd = fdCar;
  //~ };
  Armor_Detector();
  bool InitCamera();
  void set2Zero();
  void angleFilter(Point2f angle);
  bool DetectorPretreatment(Mat src, vector<cv::RotatedRect> &lights);
  void RIODetector(vector<cv::RotatedRect> &lights,
                   vector<RotatedRect> &ArmorBuild);
  void rioTactics(vector<RotatedRect> finalArmorBuild);
  void targetFilter(RotatedRect targetArmor, RotatedRect lastTargetarmor);
  void finalImgshow(RotatedRect ArmorBuild);
  Point2f dis2Angle(RotatedRect targetArmor);
  void ImageProducer();
  void ImageConsumer();
};

namespace Kalman_struct {
class KalmanFilter {
 public:
  KalmanFilter(int x, int y) : KF_(4, 2) {
    measurement_ = Mat::zeros(2, 1, CV_32F);  // (x, y)
    KF_.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1,
                            0,           //**Latter 1: Larger, faster regression
                            0, 1, 0, 1,  //**Latter 1: Larger, faster regression
                            0, 0, 1, 0, 0, 0, 0, 1);
    setIdentity(KF_.measurementMatrix, Scalar::all(1));
    setIdentity(KF_.processNoiseCov,
                Scalar::all(1e-10));  //**10: Larger, slower regression
    setIdentity(KF_.measurementNoiseCov,
                Scalar::all(1e-1));  // 1: Larger, quicker regression
    setIdentity(KF_.errorCovPost, Scalar::all(1));

    KF_.statePost =
        (Mat_<float>(4, 1) << x, y, 0, 0);  // Ensure beginner is default value
  }

  Point2f run(float x, float y) {
    Mat prediction = KF_.predict();
    Point2f predict_pt =
        Point2f(prediction.at<float>(0), prediction.at<float>(1));

    measurement_.at<float>(0, 0) = x;
    measurement_.at<float>(1, 0) = y;

    KF_.correct(measurement_);

    return predict_pt;
  }

 private:
  Mat measurement_;
  cv::KalmanFilter KF_;  // Differ from Kalman_example::KalmanFilter
};
}  // namespace Kalman_struct
