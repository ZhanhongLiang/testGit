/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-11-26 20:32:07
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-26 21:51:09
 */
#include <unistd.h>
#include <iostream>
#include <thread>
//#include "SerialPort.h"
#include "Armor_Detector.h"

using namespace std;

int main() {
  // char *dev = (char *)"/dev/ttyTHS2";  // Uart3
  // int fd;

  Armor_Detector armor;
  // portConfig(&armor.fd, dev);
  // VideoCapture capture_m(device);
  // Armor_Detector(fd);
  armor.InitCamera();
  while (1) {
    std::thread t1(&Armor_Detector::ImageProducer, armor);  //����
    std::thread t2(&Armor_Detector::ImageConsumer, armor);
    // std::thread t3(&Armor_Detector::imgThreadrun, armor);   //uart�� �д���
    t1.join();
    t2.join();
    // t3.join();
  }
  return 0;
}
