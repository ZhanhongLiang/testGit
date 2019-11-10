/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-09-05 21:23:44
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-10 18:01:22
 */
#define BUFF_DEBUG

#ifdef BUFF_DEBUG

#include <iostream>
#include <string>
#include <thread>
#include "../../include/rune/ImageConsProd.h"
#include "../../include/rune/Rune.h"
#include "../../include/rune/Settings.hpp"
//#include "../rune/AngleSolver.hpp"
//#include "Settings.hpp"

int main() {
  Buff_Detector buff_Detector;
  // AngleSolverFactory angleSlover;
  // Settings settings("D:/gitrepo/Robobigsymbol/src/rune/rune/Rune.xml");
  // angleSlover.SetTargetSize(122, 233, AngleSolverFactory::TARGET_RUNE);
  cv::Mat frame, bin_img;
  int tSelect = 3;
  int status = 1;
  int mode = 0;
  double angle_x;
  double angle_y;
  double dist;

  /**
   * @brief:
   * @param {type}
   * @return:
   * @author: Chinwong_Leung
   */
  // int mode = 1;

  VideoCapture capture;
  Point2f pt;
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