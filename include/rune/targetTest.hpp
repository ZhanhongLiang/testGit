/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-11-21 18:02:44
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-21 18:23:49
 */
#ifndef TARGETTEST_H_
#define TARGETTEST_H_
#include "Rune.h"
#include "header/Header.h"
class TargetTest {
 public:
  detect::armor::Buff_Detector *buffDetector;
  TargetTest(detect::armor::Buff_Detector buffDetector_) {
    buffDetector = buffDetector_;
  }
};
#endif