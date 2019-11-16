/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-09-05 21:23:44
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-16 19:40:32
 */
#define BUFF_DEBUG

#ifdef BUFF_DEBUG

#include <iostream>
#include <string>
#include <thread>
//#include "../../include/rune/ImageConsProd.h"
#include "../../include/rune/AngleSolver.hpp"
#include "../../include/rune/Rune.h"
using namespace std;
string file = "C:/Users/25212/Documents/RM_BUFF/config/param_config.xml";
cv::Mat cameraMatrix1;
cv::Mat distCoeffs1;

int main() {
  detect::armor::Buff_Detector buff_Detector;
  detect::angle_solver::Flag flag1;
  detect::angle_solver::Settings settings_file(file);
  detect::angle_solver::AngleSolver angle(&settings_file, &flag1);
  angle.GetCamerParam();

  // Settings settings_file(file);
  // AngleSolver angleSlover(settings_file);
  // Settings settings("D:/gitrepo/Robobigsymbol/src/rune/rune/Rune.xml");
  // angleSlover.SetTargetSize(122, 233, AngleSolverFactory::TARGET_RUNE);
  cv::Mat frame, bin_img;
  cv::RotatedRect finalArmor;
  int tSelect = 3;
  int status = 1;
  int mode = 0;
  double angle_x;
  double angle_y;
  double dist;
  // int mode = 1;

  VideoCapture capture;
  Point2f pt = Point2f(0, 0);
  int frameCount = 0;
  capture.open("C:\\Users\\25212\\Documents\\Buff\\Buff.mp4");
  if (!capture.isOpened()) {
    std::cout << "could not find the mp4!!" << std::endl;
  } else {
    while (true) {
      capture >> frame;
      if (frame.empty()) {
        std::cout << "Not the frame!!" << std::endl;
        break;
      }
      buff_Detector.Detect(frame, tSelect, pt, status);

      // angle.GetRuneAngle(finalArmor);
      // angleSlover(buff_Detector.finalArmor2Angle,
      // angleSlover.TARGET_RUNE ,angle_x, angle_y, dist);
      frameCount++;
      std::cout << "frameCount:" << frameCount << std::endl;
    }
  }
}

// int main()
// {
// 	int fd = 1;
// 	string ParamConfig =
// "D:\\gitrepo\\Robobigsymbol\\src\\rune\\rune\\camera-RM3-06-640.xml";
// 	Settings setting_file(ParamConfig);

// 	ImageConsProd imagePro(&setting_file, fd);
// 	std::thread t1(&ImageConsProd::ImageProducer, imagePro);
// 	std::thread t2(&ImageConsProd::ImageConsumer, imagePro);

// 	t1.join();
// 	t2.join();

// 	//getchar();
// 	//system("pause");
// 	return 0;
// }
#endif  // BUFF_DEBUG